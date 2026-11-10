#!/usr/bin/env python3
"""맥에서 앱 화면 미리보기 (ROS·가상머신 없이). 가짜 방·로봇·라이다·사람을 만들어 앱 서버와 같은 데이터를 보낸다.

  python3 tools/app_preview.py            # 브라우저에서 http://localhost:8765 → 조이스틱 운전, 코스 골라 순찰
  python3 tools/app_preview.py --patrol   # '터미널에서 켠 순찰' 흉내 (앱에서 멈출 수 없고 수동 주행이 잠기는지 보기)
  python3 tools/app_preview.py --port 9000

화면을 고칠 때 가상머신을 켜지 않고 바로 확인하는 용도다. 수동 주행은 실제 앱 서버와 같은 안전 규칙
(app_logic.ManualDrive, guard_speed)을 쓴다.
"""

import argparse
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'src', 'security_patrol')
sys.path.insert(0, os.path.join(PKG, 'security_patrol'))
from app_logic import ManualDrive, ObjectBoard, build_state, encode_map, guard_speed, scan_to_points  # noqa: E402
from course_planner import plan_course  # noqa: E402
from explore_logic import choose_goal, known_area  # noqa: E402
from web_server import WebServer  # noqa: E402

# 가짜 방: 6 × 4 m 벽 + 가운데 상자 + 칸막이 (선분 목록)
WALLS = [((-3, -2), (3, -2)), ((3, -2), (3, 2)), ((3, 2), (-3, 2)), ((-3, 2), (-3, -2)),
         ((0.8, 0.4), (1.4, 0.4)), ((1.4, 0.4), (1.4, 1.0)), ((1.4, 1.0), (0.8, 1.0)), ((0.8, 1.0), (0.8, 0.4)),
         ((-1.0, -2.0), (-1.0, -0.8))]
RES, ORIGIN = 0.05, (-3.3, -2.3)
MAP_W, MAP_H = int(6.6 / RES), int(4.6 / RES)


def ray(x, y, a, walls, extra=()):
    """(x, y) 에서 a 방향으로 쏜 빛이 처음 닿는 거리. extra 는 원 (cx, cy, r) 목록(사람 다리)."""
    dx, dy = math.cos(a), math.sin(a)
    best = math.inf
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    for cx, cy, r in extra:
        fx, fy = x - cx, y - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                best = min(best, t)
    return best


def make_map():
    data = []
    for j in range(MAP_H):
        for i in range(MAP_W):
            x, y = ORIGIN[0] + (i + 0.5) * RES, ORIGIN[1] + (j + 0.5) * RES
            near = any(dist_to_segment(x, y, *w) < RES for w in WALLS)
            inside = -3 <= x <= 3 and -2 <= y <= 2
            data.append(100 if near else 0 if inside else -1)
    return encode_map(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], 0.0, data), data


def dist_to_segment(px, py, a, b):
    (x1, y1), (x2, y2) = a, b
    ex, ey = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (x1 + t * ex), py - (y1 + t * ey))


def fake_scan(rx, ry, ryaw, people, step):
    ranges = []
    for i in range(720):
        ang = -math.pi + i * (2 * math.pi / 720)
        r = ray(rx, ry, ryaw + ang, WALLS, people)
        ranges.append(r if r <= 8.0 else math.inf)
    return scan_to_points(ranges, -math.pi, 2 * math.pi / 720, 0.15, 8.0, step=step)


# 가짜 코스 (가짜 방 안, 벽·상자를 피해서 잡음)
COURSES = [
    {'id': 'preview_loop', 'name': '미리보기 한 바퀴', 'dwell': 2.0,
     'points': [{'name': '출발점', 'x': -2.0, 'y': 0.3}, {'name': '남서쪽', 'x': -2.2, 'y': -1.3},
                {'name': '가운데', 'x': -0.3, 'y': -0.3}, {'name': '남동쪽', 'x': 2.2, 'y': -1.2},
                {'name': '북동쪽', 'x': 2.3, 'y': 1.4}, {'name': '북쪽', 'x': 0.0, 'y': 1.4}]},
    {'id': 'preview_short', 'name': '가운데 왕복', 'dwell': 3.0,
     'points': [{'name': '서쪽', 'x': -1.8, 'y': 0.6}, {'name': '동쪽', 'x': 0.4, 'y': 0.0}]},
]


class FakePatrol:
    """patrol_node 흉내: 지점으로 돌고 가서 dwell 초 멈추고, 정한 바퀴를 돌면 끝난다."""

    def __init__(self, course, laps, owner):
        self.course, self.laps, self.owner = course, laps, owner
        self.lap, self.index, self.state, self.until = 1, 0, 'moving', 0.0
        self.stop_at = None

    def status(self):
        p = self.course['points'][self.index]
        return {'state': self.state, 'lap': self.lap, 'index': self.index, 'name': p['name'], 'x': p['x'], 'y': p['y']}

    def step(self, pose, t, dt):
        """새 위치와 끝났는지(None / 'done') 를 돌려준다."""
        x, y, yaw = pose
        if self.stop_at is not None:
            return pose, ('stopped' if t >= self.stop_at else None)
        p = self.course['points'][self.index]
        if self.state == 'watching':
            if t < self.until:
                return pose, None
            self.index += 1
            if self.index == len(self.course['points']):
                self.index = 0
                self.lap += 1
                if self.laps and self.lap > self.laps:
                    return pose, 'done'
            self.state = 'moving'
            return pose, None
        dx, dy = p['x'] - x, p['y'] - y
        dist = math.hypot(dx, dy)
        if dist < 0.06:
            self.state, self.until = 'watching', t + self.course['dwell']
            return pose, None
        err = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        w = max(-1.5, min(1.5, 3.0 * err))
        v = 0.0 if abs(err) > 0.4 else min(0.3, dist)
        yaw += w * dt
        return (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw), None


class FakeExplorer:
    """explorer 흉내: 빈 지도에서 시작해 가짜 방을 라이다(ray)로 조금씩 밝히며 explore_logic 으로 목표를 고른다."""

    def __init__(self, home, t):
        self.known = [-1] * (MAP_W * MAP_H)
        self.home, self.started = home, t
        self.goal, self.blacklist, self.visited = None, [], []
        self.state, self.goals, self.frontiers, self.until = 'starting', 0, None, t + 2.0

    def reveal(self, x, y, rng=3.0):
        for k in range(0, 360, 2):
            a = math.radians(k)
            hit = ray(x, y, a, WALLS)
            n = int(min(hit, rng) / (RES * 0.5))
            for s in range(1, n + 1):
                c = int((x + math.cos(a) * s * RES * 0.5 - ORIGIN[0]) / RES)
                r = int((y + math.sin(a) * s * RES * 0.5 - ORIGIN[1]) / RES)
                if 0 <= c < MAP_W and 0 <= r < MAP_H and self.known[r * MAP_W + c] != 100:
                    self.known[r * MAP_W + c] = 0
            if hit <= rng:
                c = int((x + math.cos(a) * (hit + RES * 0.3) - ORIGIN[0]) / RES)
                r = int((y + math.sin(a) * (hit + RES * 0.3) - ORIGIN[1]) / RES)
                if 0 <= c < MAP_W and 0 <= r < MAP_H:
                    self.known[r * MAP_W + c] = 100

    def status(self, t):
        return {'state': self.state, 'known_area': round(known_area(self.known, RES), 2), 'goals': self.goals,
                'failed': 0, 'abandoned': len(self.blacklist), 'frontiers': self.frontiers,
                'elapsed': round(t - self.started), 'goal': list(self.goal) if self.goal else None}

    def step(self, pose, t, dt):
        x, y, yaw = pose
        if self.state == 'starting':
            if t >= self.until:
                self.state = 'exploring'
                self.reveal(x, y)
            return pose
        if self.state in ('saving', 'done'):
            if self.state == 'saving' and t >= self.until:
                self.state = 'done'
            return pose
        if self.goal is None and self.state == 'exploring':
            pick = choose_goal(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], self.known, (x, y), self.blacklist, self.visited)
            self.blacklist += pick['abandon']
            self.frontiers = pick['frontiers']
            if pick['goal'] is None:
                self.state, self.goal = 'returning', self.home[:2]
            else:
                self.goal = pick['goal']
                self.goals += 1
        gx, gy = self.goal
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.05:
            if self.state == 'returning':
                self.state, self.until = 'saving', t + 1.5
            else:
                self.visited.append(self.goal)
            self.goal = None
            return (x, y, yaw)
        yaw = math.atan2(gy - y, gx - x)
        step = min(dist, 0.5 * dt)                 # 미리보기라 실제보다 빠르게 (0.5 m/s)
        x, y = x + step * math.cos(yaw), y + step * math.sin(yaw)
        self.reveal(x, y)
        return (x, y, yaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--patrol', action='store_true', help="'터미널에서 켠 순찰' 흉내 (앱에서 못 멈춤)")
    args = ap.parse_args()
    map_msg, map_data = make_map()
    plans = {}
    drive, lock = ManualDrive(max_linear=0.3, max_angular=1.2), threading.Lock()
    sim = {'patrol': FakePatrol(COURSES[0], 0, 'external') if args.patrol else None, 'last': None, 'status': None,
           # 모드 흉내: 순찰 / 지도 만들기. 바꾸는 데 2초 걸리는 척한다
           'mode': 'patrol', 'busy_until': 0.0, 'next': None, 'explorer': None, 'map_kind': 'default',
           'explored': None, 'map': map_data, 'map_sent': 0.0}

    def map_message(data):
        return encode_map(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], 0.0, data)

    def mode_state(t):
        ex = sim['explorer']
        exploring = sim['mode'] == 'explore' and ex is not None and ex.state != 'done'
        return {'managed': True, 'mode': None if t < sim['busy_until'] else sim['mode'],
                'busy': t < sim['busy_until'], 'crashed': False, 'exploring': exploring,
                'map': {'default': 'fake_room.yaml', 'explored': 'explored.yaml'}[sim['map_kind']]
                if sim['mode'] == 'patrol' else None,
                'map_kind': sim['map_kind'], 'explored_ready': sim['explored'] is not None,
                'explore': ex.status(time.time() - t0) if sim['mode'] == 'explore' and ex else None}

    def notice(cid, text, level='warn'):
        web.send_to(cid, {'type': 'notice', 'level': level, 'text': text})

    def on_message(cid, msg):                       # 웹 서버 스레드
        kind = msg.get('type')
        with lock:
            patrol = sim['patrol']
            if kind == 'disconnected':
                drive.disconnect(cid)
            elif kind == 'stop':
                drive.stop()
            elif kind in ('explore_start', 'map_use'):
                now = time.time() - t0
                if now < sim['busy_until']:
                    notice(cid, '모드를 바꾸는 중입니다. 잠시 기다려 주세요')
                elif kind == 'explore_start':
                    sim['patrol'] = None
                    drive.stop('mode')
                    sim['busy_until'], sim['next'] = now + 2.0, ('explore', None)
                elif msg.get('which') == 'explored' and sim['explored'] is None:
                    notice(cid, '탐사로 만든 지도가 아직 없습니다')
                else:
                    which = 'explored' if msg.get('which') == 'explored' else 'default'
                    drive.stop('mode')
                    sim['busy_until'], sim['next'] = now + 2.0, ('patrol', which)
            elif kind == 'drive' and patrol is None and not mode_state(time.time() - t0)['exploring'] \
                    and time.time() - t0 >= sim['busy_until']:
                drive.command(cid, msg.get('lin'), msg.get('ang'), time.monotonic())
            elif kind == 'patrol_start':
                course = next((c for c in COURSES if c['id'] == msg.get('course')), None)
                if sim['mode'] == 'explore' or time.time() - t0 < sim['busy_until']:
                    notice(cid, '지도 만들기 모드에서는 순찰할 수 없습니다')
                elif patrol is not None:
                    notice(cid, '이미 순찰 중입니다')
                elif course is None:
                    notice(cid, '없는 코스입니다')
                else:
                    drive.stop('patrol')
                    sim['patrol'], sim['status'] = FakePatrol(course, int(msg.get('laps', 0)), 'app'), None
                    web.broadcast({'type': 'notice', 'level': 'ok', 'text': '순찰을 시작합니다'})
            elif kind == 'patrol_stop':
                if patrol is None:
                    pass
                elif patrol.owner != 'app':
                    notice(cid, '터미널에서 켠 순찰입니다. 그 터미널에서 Ctrl+C 로 멈추세요')
                elif patrol.stop_at is None:
                    patrol.stop_at = time.time() - t0 + 1.5          # 이동 취소에 걸리는 시간 흉내
                    web.broadcast({'type': 'notice', 'level': 'ok', 'text': '순찰을 멈추는 중입니다 (이동 취소)'})
            elif kind == 'course_plan':                 # 실제 앱 서버와 같은 계산 (course_planner)
                try:
                    current = sim['explorer'].known if sim['mode'] == 'explore' and sim['explorer'] else sim['map']
                    plan = plan_course(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], current, pose[:2],
                                       view_range=float(msg.get('view', 2.5)))
                    plan.update(id='__plan__', name='자동 코스 (미리보기)', dwell=2.0)
                    plans[cid] = plan
                    web.send_to(cid, {'type': 'course_plan', 'plan': plan})
                except ValueError as e:
                    notice(cid, f'코스를 만들지 못했습니다: {e}', 'bad')
            elif kind == 'course_save' and cid in plans:   # 미리보기에서는 파일 대신 메모리에만 저장
                plan = plans.pop(cid)
                cid_new = f'auto_{len(COURSES)}'
                COURSES.append({'id': cid_new, 'name': msg.get('name') or f'자동 코스 {len(COURSES)}',
                                'dwell': 2.0, 'points': plan['points']})
                web.broadcast({'type': 'courses', 'courses': COURSES})
                web.send_to(cid, {'type': 'course_saved', 'id': cid_new})
                notice(cid, '코스를 저장했습니다 (미리보기: 메모리에만)', 'ok')
            elif kind == 'courses':
                web.send_to(cid, {'type': 'courses', 'courses': COURSES})
        if kind not in ('drive',):
            print(f'앱 #{cid} → {msg}')

    def on_connect(cid):
        web.send_to(cid, {'type': 'hello', 'id': cid})
        web.send_to(cid, {'type': 'courses', 'courses': COURSES})
        if sim['mode'] == 'explore' and sim['explorer']:
            web.send_to(cid, map_message(sim['explorer'].known))
        elif time.time() - t0 >= sim['busy_until']:
            web.send_to(cid, map_message(sim['map']))

    web = WebServer(os.path.join(PKG, 'web'), host='127.0.0.1', port=args.port,
                    on_message=on_message, on_connect=on_connect).start_in_thread()
    print(f'미리보기: http://localhost:{web.port}  (Ctrl+C 로 종료)')
    board = ObjectBoard()
    pose = (-2.0, 0.3, 0.0)                         # 로봇 시작 위치
    blocked, dt, n = None, 0.1, 0
    t0 = time.time()
    while True:
        t = time.time() - t0
        rx, ry, ryaw = pose
        # 사람: 10초 뒤 들어와서 천천히 걸음 (다리 두 개)
        people = []
        if t > 10:
            px, py = -2.2 + 0.25 * ((t - 10) % 16), 1.2
            people = [(px, py + 0.1, 0.06), (px, py - 0.1, 0.06)]
            board.update({'event': 'moving' if t > 16 else 'detected', 'id': 1, 'x': px, 'y': py,
                          'width': 0.3, 'size': 'medium', 'moving': t > 16, 'person': t > 13}, t)
        with lock:
            patrol = sim['patrol']
            if patrol is not None:                  # 순찰: 코스를 따라 움직임
                pose, end = patrol.step(pose, t, dt)
                sim['status'] = patrol.status()
                if end:
                    sim['last'] = {'course': patrol.course['id'], 'code': 0, 'stopped': end == 'stopped'}
                    sim['patrol'] = None
                    print(f'순찰 끝남: {end}')
            if sim['next'] and t >= sim['busy_until']:        # 모드 바꾸기 끝 → 새 모드 시작
                mode, which = sim['next']
                sim['next'], sim['mode'], sim['status'] = None, mode, None
                board = ObjectBoard()
                if mode == 'explore':
                    sim['explorer'] = FakeExplorer(pose, t)
                else:
                    sim['explorer'], sim['map_kind'] = None, which
                    sim['map'] = sim['explored'] if which == 'explored' else map_data
                    web.broadcast(map_message(sim['map']))
                web.broadcast({'type': 'notice', 'level': 'ok',
                               'text': '지도 만들기 모드 시작' if mode == 'explore' else '순찰 모드 시작'})
            ex = sim['explorer']
            if sim['mode'] == 'explore' and ex is not None and not sim['next']:
                was = ex.state
                pose = ex.step(pose, t, dt)
                if t - sim['map_sent'] > 1.0:               # 탐사 중에는 지도가 자라는 모습을 1초마다
                    sim['map_sent'] = t
                    web.broadcast(map_message(ex.known))
                if ex.state == 'done' and was != 'done':    # 지도 저장 + 탐사 지도로 코스 만들기
                    sim['explored'] = list(ex.known)
                    web.broadcast(map_message(ex.known))
                    try:
                        plan = plan_course(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], ex.known, ex.home[:2])
                        COURSES[:] = [c for c in COURSES if c['id'] != 'explored_course']
                        COURSES.append({'id': 'explored_course', 'name': '탐사로 만든 코스', 'dwell': 2.0,
                                        'points': plan['points']})
                        web.broadcast({'type': 'courses', 'courses': COURSES})
                    except ValueError as e:
                        print(f'탐사 지도로 코스를 못 만듦: {e}')
                    print(f'탐사 끝: {ex.goals}곳, {known_area(ex.known, RES):.1f} m²')
            out = drive.step(time.monotonic())
        blocked = None
        if patrol is None and out is not None:      # 조이스틱 명령대로 움직임 (서버와 같은 규칙)
            v, w = out
            v, blocked = guard_speed(v, fake_scan(rx, ry, ryaw, people, 1))
            nyaw = ryaw + w * dt
            nx, ny = rx + v * math.cos(nyaw) * dt, ry + v * math.sin(nyaw) * dt
            if all(dist_to_segment(nx, ny, *wall) > 0.12 for wall in WALLS):
                pose = (nx, ny, nyaw)
        n += 1
        if n % 2 == 0:                              # 상태는 5 Hz
            with lock:
                manual = drive.snapshot()
                patrol = sim['patrol']
                running = patrol is not None
                patrol_app = {'running': running and patrol.owner == 'app',
                              'course': patrol.course['id'] if running and patrol.owner == 'app' else None,
                              'laps': patrol.laps if running else None,
                              'stopping': running and patrol.stop_at is not None,
                              'owner': patrol.owner if running else None, 'last': sim['last']}
                status = sim['status']
                mode = mode_state(t)
            manual['blocked'] = blocked
            rx, ry, ryaw = pose
            objects = [] if mode['mode'] != 'patrol' else board.snapshot(t)   # 감지는 순찰 모드에서만
            web.broadcast(build_state(pose, fake_scan(rx, ry, ryaw, people, 2), 7.38, -54, status,
                                      objects, True, t, manual=manual, patrol_running=running,
                                      patrol_app=patrol_app, mode=mode))
        time.sleep(dt)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n종료')
