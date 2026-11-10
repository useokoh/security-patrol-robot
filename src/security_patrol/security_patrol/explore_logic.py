"""자율 탐사 계산: 프런티어(아는 빈칸과 모르는 칸의 경계)를 찾아 다음 목표를 고른다.

ROS·numpy 없이 시험할 수 있다 (test/test_explore_logic.py). explorer 노드가 이 결과로 Nav2 목표를 보낸다.

  1. 프런티어 칸 = 빈칸 중 상하좌우에 모르는 칸(-1)이 있는 칸. 8방향으로 이어진 것끼리 덩어리로 묶고,
     min_size 보다 작은 덩어리(라이다 잡음, 틈)는 버린다.
  2. 관찰 지점 = 덩어리 근처(look_radius 안)의 아는 빈칸 중, 벽에서 goal_clearance 이상 떨어지고
     로봇이 아는 빈칸만 지나 갈 수 있는 곳. 그중 로봇에서 가장 가까운 곳.
     (모르는 칸 위를 목표로 주면 Nav2 가 벽 속을 목표로 받을 수 있어서, 이미 아는 곳에서 바라보게 한다)
  3. 점수 = 이동 거리 - size_weight x 덩어리 길이 → 가깝고 큰 경계부터. 실패했던 곳 근처는 건너뛴다.
  4. 가장 가까운 관찰 지점이 로봇이 서 있는 자리(min_move 안)이거나 이미 가 본 자리라면, 거기서는 그 경계가 안 보인다는 뜻이다
     (문틀에 가림, 라이다 거리 밖). 그러면 덩어리 가운데에 가장 가까운 다른 관찰 지점(예: 문을 지나 방 안)을
     한 번 더 고른다. 이미 가 본 관찰 지점(visited)은 다시 고르지 않는다 (두 곳 사이 왕복 방지).
     그것도 없으면 볼 수 없는 경계(지도 밖 틈, 유리)로 보고 'abandon' 으로 돌려주며,
     부르는 쪽이 포기 목록에 넣는다. 포기하지 않으면 같은 자리를 계속 골라 탐사가 멈춘다 (맥 가짜 탐사에서 발견).
  5. 고를 덩어리가 없으면 탐사 끝 (goal None).
"""

import math

try:
    from security_patrol.course_planner import FREE_MAX, GridMap, clearance_map, path_lengths, snap_start
except ImportError:      # 시험·맥 도구: security_patrol 폴더를 바로 sys.path 에 넣은 경우
    from course_planner import FREE_MAX, GridMap, clearance_map, path_lengths, snap_start


def frontier_clusters(width, height, data, min_cells):
    """프런티어 칸을 8방향으로 이어진 덩어리 목록으로 ([[칸 번호...], ...])."""
    w, h = width, height
    free = [0 <= v <= FREE_MAX for v in data]
    unknown = [v < 0 for v in data]
    is_frontier = [False] * (w * h)
    for i in range(w * h):
        if not free[i]:
            continue
        r, c = divmod(i, w)
        if ((c > 0 and unknown[i - 1]) or (c < w - 1 and unknown[i + 1])
                or (r > 0 and unknown[i - w]) or (r < h - 1 and unknown[i + w])):
            is_frontier[i] = True
    clusters, seen = [], [False] * (w * h)
    for i in range(w * h):
        if not is_frontier[i] or seen[i]:
            continue
        group, stack = [], [i]
        seen[i] = True
        while stack:
            j = stack.pop()
            group.append(j)
            r, c = divmod(j, w)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w:
                        k = rr * w + cc
                        if is_frontier[k] and not seen[k]:
                            seen[k] = True
                            stack.append(k)
        if len(group) >= min_cells:
            clusters.append(group)
    return clusters


def known_area(data, resolution):
    """아는 빈칸 넓이 [m²]."""
    return sum(1 for v in data if 0 <= v <= FREE_MAX) * resolution * resolution


def choose_goal(width, height, resolution, origin_x, origin_y, data, robot_xy, blacklist=(), visited=(),
                robot_radius=0.17, goal_clearance=0.30, look_radius=0.8, min_size=0.3,
                size_weight=1.0, blacklist_radius=0.5, min_move=0.25):
    """다음 탐사 목표.

    돌려주는 값: {'goal': (x, y) 또는 None(탐사 끝), 'look': (x, y) 덩어리 가운데 (도착 방향),
                 'size': 덩어리 길이 [m], 'distance': 이동 거리 [m], 'frontiers': 남은 덩어리 수,
                 'abandon': [(x, y)...] 이번에 포기한 관찰 지점 (부르는 쪽이 blacklist 에 더한다)}
    """
    done = {'goal': None, 'look': None, 'size': 0.0, 'distance': 0.0, 'frontiers': 0, 'abandon': []}
    res = resolution
    clusters = frontier_clusters(width, height, data, max(1, int(round(min_size / res))))
    if not clusters:
        return done
    # 벽까지 거리는 '막힌 칸'만 기준 (모르는 칸은 벽이 아니다). 이동은 아는 빈칸으로만
    known = GridMap(width, height, res, origin_x, origin_y, [0 if v < 0 else v for v in data])
    clear = clearance_map(known)
    m = GridMap(width, height, res, origin_x, origin_y, data)
    passable = [m.free[i] and clear[i] >= robot_radius for i in range(width * height)]
    src = snap_start(m, clear, robot_xy, robot_radius)
    if src is None:
        return done
    dist = path_lengths(m, passable, src)
    abandon = []
    w, h, reach_cells = width, height, int(math.ceil(look_radius / res))

    best = None
    for group in clusters:
        gx = sum(m.xy_of(i)[0] for i in group) / len(group)
        gy = sum(m.xy_of(i)[1] for i in group) / len(group)
        # 덩어리 칸에서 look_radius 안쪽으로 퍼지며 관찰 지점 후보를 찾는다 (막힌 칸은 넘지 않음)
        reach = {i: 0 for i in group}
        frontier = list(group)
        for step in range(1, reach_cells + 1):
            nxt = []
            for i in frontier:
                r, c = divmod(i, w)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w:
                        j = rr * w + cc
                        if j not in reach and data[j] <= FREE_MAX:
                            reach[j] = step
                            nxt.append(j)
            frontier = nxt
        cands = [i for i in reach if passable[i] and clear[i] >= goal_clearance and dist[i] < float('inf')]
        if not cands:
            continue
        def fresh(i):                             # 아직 가 보지 않은 관찰 지점인지
            x, y = m.xy_of(i)
            return all(math.hypot(x - vx, y - vy) >= min_move for vx, vy in visited)

        goal = min(cands, key=lambda i: dist[i])
        # 지금 서 있거나 이미 가 본 자리인데도 경계가 남았다면 거기서는 안 보인다 → 덩어리 쪽 다른 관찰 지점
        if dist[goal] < min_move or not fresh(goal):
            farther = [i for i in cands if dist[i] >= min_move and fresh(i)]
            if not farther:                       # 다른 곳도 없으면 볼 수 없는 경계 → 포기
                x, y = m.xy_of(goal)
                abandon.append((round(x, 2), round(y, 2)))
                continue
            goal = min(farther, key=lambda i: (m.xy_of(i)[0] - gx) ** 2 + (m.xy_of(i)[1] - gy) ** 2)
        x, y = m.xy_of(goal)
        if any(math.hypot(x - bx, y - by) < blacklist_radius for bx, by in list(blacklist) + abandon):
            continue
        size = len(group) * res
        score = dist[goal] - size_weight * size
        if best is None or score < best[0]:
            best = (score, {'goal': (round(x, 2), round(y, 2)), 'look': (round(gx, 2), round(gy, 2)),
                            'size': round(size, 2), 'distance': round(dist[goal], 2)})
    if best is None:
        return dict(done, frontiers=len(clusters), abandon=abandon)
    best[1].update(frontiers=len(clusters), abandon=abandon)
    return best[1]


def frontier_near(width, height, resolution, origin_x, origin_y, data, point, radius, min_size=0.3):
    """point 근처(radius 안)에 아직 프런티어 덩어리가 남아 있는지 (가는 도중 이미 보였으면 False)."""
    clusters = frontier_clusters(width, height, data, max(1, int(round(min_size / resolution))))
    m = GridMap(width, height, resolution, origin_x, origin_y, data)
    px, py = point
    for group in clusters:
        for i in group:
            x, y = m.xy_of(i)
            if (x - px) ** 2 + (y - py) ** 2 <= radius * radius:
                return True
    return False
