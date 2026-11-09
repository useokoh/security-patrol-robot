"""지도에서 순찰 코스를 자동으로 만든다. ROS·numpy 없이 시험할 수 있다 (test/test_course_planner.py).

사람이 지점마다 route_recorder 로 기록하던 일을 대신한다. 순서:
  1. 안전 거리: 칸마다 가장 가까운 벽(또는 모르는 곳)까지의 거리 (chamfer 거리 변환)
  2. 갈 수 있는 곳: 로봇 위치에서 로봇 반지름보다 넓은 칸만 따라 퍼져 나간 영역
  3. 지점 고르기 (그리디 커버리지): 벽에서 clearance 이상 떨어진 후보 중, 라이다(view_range)로
     '아직 안 보인 빈 공간'을 가장 많이 새로 보는 곳부터 하나씩 고른다. 보이는 비율이 target 을
     넘거나 새로 보이는 공간이 너무 적으면 멈춘다 → 넓게 트인 방은 지점이 적고, 칸막이가 많으면 늘어난다
  5. 순서: 벽을 돌아가는 실제 이동 거리로 가까운 곳부터 잇고(최근접 이웃), 2-opt 로 꼬인 곳을 편다
  6. 이름: 빈 공간 전체를 3x3 으로 나눈 위치 (북서쪽 … 가운데 … 남동쪽). 알림의 '어느 지점 근처'에 쓰인다

지도 값: 0~FREE_MAX 빈칸, 그 밖(벽·모름 -1)은 막힌 칸. 지도 원점 회전은 0 이라고 가정한다 (slam_toolbox·map_saver 기본).
"""

import heapq
import math

FREE_MAX = 25            # 이 값 이하만 빈칸 (map_server free_thresh 0.25 와 맞춤)
SQRT2 = math.sqrt(2.0)
NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]
NAMES = [['남서쪽', '남쪽', '남동쪽'], ['서쪽', '가운데', '동쪽'], ['북서쪽', '북쪽', '북동쪽']]


class GridMap:
    """OccupancyGrid 를 다루기 쉽게 감싼 것. 칸 번호 i = row * width + col (row 0 이 지도 아래쪽)."""

    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        if len(data) != width * height:
            raise ValueError('지도 크기와 데이터 길이가 다릅니다')
        self.w, self.h, self.res = width, height, resolution
        self.ox, self.oy = origin_x, origin_y
        self.free = [0 <= v <= FREE_MAX for v in data]

    def cell_of(self, x, y):
        c, r = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        return r * self.w + c if 0 <= c < self.w and 0 <= r < self.h else None

    def xy_of(self, i):
        r, c = divmod(i, self.w)
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res


def clearance_map(m):
    """칸마다 가장 가까운 막힌 칸까지의 거리 [m]. 두 번 훑는 chamfer 거리 변환 (1, √2)."""
    w, h, inf = m.w, m.h, float('inf')
    d = [inf if f else 0.0 for f in m.free]
    for r in range(h):                                    # 지도 밖도 막힌 것으로 본다
        for c in (0, w - 1):
            d[r * w + c] = min(d[r * w + c], 0.5 if m.free[r * w + c] else 0.0)
    for c in range(w):
        for r in (0, h - 1):
            d[r * w + c] = min(d[r * w + c], 0.5 if m.free[r * w + c] else 0.0)
    for r in range(h):                                    # 앞으로 훑기
        for c in range(w):
            i = r * w + c
            if d[i] == 0.0:
                continue
            if c > 0:
                d[i] = min(d[i], d[i - 1] + 1.0)
            if r > 0:
                d[i] = min(d[i], d[i - w] + 1.0)
                if c > 0:
                    d[i] = min(d[i], d[i - w - 1] + SQRT2)
                if c < w - 1:
                    d[i] = min(d[i], d[i - w + 1] + SQRT2)
    for r in range(h - 1, -1, -1):                        # 뒤로 훑기
        for c in range(w - 1, -1, -1):
            i = r * w + c
            if d[i] == 0.0:
                continue
            if c < w - 1:
                d[i] = min(d[i], d[i + 1] + 1.0)
            if r < h - 1:
                d[i] = min(d[i], d[i + w] + 1.0)
                if c < w - 1:
                    d[i] = min(d[i], d[i + w + 1] + SQRT2)
                if c > 0:
                    d[i] = min(d[i], d[i + w - 1] + SQRT2)
    return [v * m.res for v in d]


def path_lengths(m, passable, src):
    """src 칸에서 passable 칸만 지나 각 칸까지 가는 거리 [m] (다익스트라, 8방향). 못 가면 inf."""
    w, h, inf = m.w, m.h, float('inf')
    dist = [inf] * (w * h)
    dist[src] = 0.0
    heap = [(0.0, src)]
    while heap:
        dd, i = heapq.heappop(heap)
        if dd > dist[i]:
            continue
        r, c = divmod(i, w)
        for dc, dr, step in NEIGHBORS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w:
                j = rr * w + cc
                nd = dd + step * m.res
                if passable[j] and nd < dist[j]:
                    dist[j] = nd
                    heapq.heappush(heap, (nd, j))
    return dist


COARSE = 3                # 보이는지 여부는 3x3 칸(15 cm) 묶음으로 센다 → 광선 사이 빈틈을 '안 보임'으로 세지 않게


def coarse(m, i):
    r, c = divmod(i, m.w)
    return (r // COARSE, c // COARSE)


def visible_cells(m, i, view_range, ray_step_deg=3):
    """칸 i 에서 라이다로 보이는 빈 공간 (3x3 칸 묶음 집합). 막힌 칸에서 빛이 멈춘다.

    3° 간격 광선은 2.5 m 에서 약 13 cm 간격이라 15 cm 묶음을 빠짐없이 지난다.
    """
    x0, y0 = m.xy_of(i)
    seen = set()
    step = m.res * 0.7
    n = int(view_range / step)
    for k in range(0, 360, ray_step_deg):
        a = math.radians(k)
        dx, dy = math.cos(a) * step, math.sin(a) * step
        for s in range(1, n + 1):
            j = m.cell_of(x0 + dx * s, y0 + dy * s)
            if j is None or not m.free[j]:
                break
            seen.add(coarse(m, j))
    return seen


def connected_free(m, src):
    """src 칸과 빈칸으로 이어진 모든 빈칸 (4방향)."""
    w, h = m.w, m.h
    seen, stack = {src}, [src]
    while stack:
        i = stack.pop()
        r, c = divmod(i, w)
        for j, ok in ((i - 1, c > 0), (i + 1, c < w - 1), (i - w, r > 0), (i + w, r < h - 1)):
            if ok and m.free[j] and j not in seen:
                seen.add(j)
                stack.append(j)
    return seen


def snap_start(m, clear, start, robot_radius):
    """로봇 위치 칸이 좁으면 가장 가까운 넓은 칸으로 옮긴다. 없으면 None."""
    i = m.cell_of(*start)
    if i is None:
        return None
    if clear[i] >= robot_radius:
        return i
    best, best_d = None, float('inf')
    for j, cl in enumerate(clear):
        if cl >= robot_radius:
            x, y = m.xy_of(j)
            d = (x - start[0]) ** 2 + (y - start[1]) ** 2
            if d < best_d:
                best, best_d = j, d
    return best if best_d <= 1.0 else None                # 1 m 넘게 떨어지면 지도와 위치가 안 맞는 것


def order_tour(dist):
    """dist[i][j] 이동 거리로 0번에서 시작하는 한 바퀴 순서. 최근접 이웃 + 2-opt."""
    n = len(dist)
    tour, left = [0], set(range(1, n))
    while left:
        nxt = min(left, key=lambda j: dist[tour[-1]][j])
        tour.append(nxt)
        left.remove(nxt)
    improved = True
    while improved and n > 3:
        improved = False
        for a in range(1, n - 1):
            for b in range(a + 1, n):
                i, j = tour[a - 1], tour[a]
                k, m_ = tour[b], tour[(b + 1) % n]
                if dist[i][k] + dist[j][m_] < dist[i][j] + dist[k][m_] - 1e-9:
                    tour[a:b + 1] = reversed(tour[a:b + 1])
                    improved = True
    return tour


def tour_length(dist, tour):
    return sum(dist[tour[k]][tour[(k + 1) % len(tour)]] for k in range(len(tour)))


def name_points(points, bounds):
    """빈 공간 범위를 3x3 으로 나눠 이름을 붙인다. 겹치면 '북쪽 2' 처럼 번호."""
    (x0, y0), (x1, y1) = bounds
    used = {}
    for p in points:
        col = min(2, max(0, int(3 * (p['x'] - x0) / max(x1 - x0, 1e-6))))
        row = min(2, max(0, int(3 * (p['y'] - y0) / max(y1 - y0, 1e-6))))
        base = NAMES[row][col]
        used[base] = used.get(base, 0) + 1
        p['name'] = base if used[base] == 1 else f'{base} {used[base]}'


def plan_course(width, height, resolution, origin_x, origin_y, data, start,
                view_range=2.5, clearance=0.35, robot_radius=0.17, target=0.97, min_gain=0.04,
                max_points=15, candidate_step=0.25):
    """지도와 로봇 위치(start=(x, y))로 순찰 코스를 만든다.

    view_range: 한 지점에서 감시한다고 보는 거리 [m]. 작을수록 지점이 많아진다.
    돌려주는 값: {'points': [{'name', 'x', 'y'}...], 'length': 한 바퀴 [m], 'coverage': 보이는 빈 공간 비율,
                 'reachable_area': 갈 수 있는 면적 [m²]}
    만들 수 없으면 ValueError(이유).
    """
    m = GridMap(width, height, resolution, origin_x, origin_y, data)
    clear = clearance_map(m)
    src = snap_start(m, clear, start, robot_radius)
    if src is None:
        raise ValueError('로봇 위치가 지도의 빈 공간에 없습니다 (위치추정을 확인하세요)')
    passable = [cl >= robot_radius for cl in clear]
    reach = path_lengths(m, passable, src)
    stride = max(1, int(round(candidate_step / m.res)))          # 후보는 candidate_step 간격으로만 (계산량)
    candidates = [i for i in range(m.w * m.h)
                  if reach[i] < float('inf') and clear[i] >= clearance
                  and (i // m.w) % stride == 0 and (i % m.w) % stride == 0]
    if not candidates:
        raise ValueError(f'벽에서 {clearance:.2f} m 이상 떨어진 갈 수 있는 곳이 없습니다')

    # 3. 그리디 커버리지: 새로 보이는 빈 공간이 가장 많은 후보부터 (같으면 더 트인 곳)
    #    감시할 공간 = 로봇이 갈 수 있는 곳과 이어진 모든 빈칸 (로봇이 못 들어가는 벽 옆·좁은 틈도 포함)
    free_reach = connected_free(m, src)
    targets = {coarse(m, i) for i in free_reach}
    views = {i: visible_cells(m, i, view_range) & targets for i in candidates}
    seen, picked = set(), []
    total = max(1, len(targets))
    while len(picked) < max_points and len(seen) < target * total:
        best = max(candidates, key=lambda i: (len(views[i] - seen), clear[i]))
        gain = len(views[best] - seen)
        if picked and gain < min_gain * total:
            break
        picked.append(best)
        seen |= views[best]

    # 5. 순서: 로봇에서 가장 가까운 지점부터, 실제 이동 거리로
    start_order = sorted(picked, key=lambda i: reach[i])
    dists = [path_lengths(m, passable, i) for i in start_order]
    matrix = [[dists[a][b] for b in start_order] for a in range(len(start_order))]
    tour = order_tour(matrix) if len(start_order) > 1 else [0]
    points = []
    for k in tour:
        x, y = m.xy_of(start_order[k])
        points.append({'name': '', 'x': round(x, 2), 'y': round(y, 2)})

    xs = [m.xy_of(i)[0] for i in free_reach]
    ys = [m.xy_of(i)[1] for i in free_reach]
    name_points(points, ((min(xs), min(ys)), (max(xs), max(ys))))
    return {'points': points,
            'length': round(tour_length(matrix, tour), 2) if len(points) > 1 else 0.0,
            'coverage': round(len(seen) / total, 3),
            'reachable_area': round(len(free_reach) * m.res * m.res, 2)}
