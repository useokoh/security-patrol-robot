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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--patrol', action='store_true', help="'터미널에서 켠 순찰' 흉내 (앱에서 못 멈춤)")
    args = ap.parse_args()
    map_msg, map_data = make_map()
    plans = {}
    drive, lock = ManualDrive(max_linear=0.3, max_angular=1.2), threading.Lock()
    sim = {'patrol': FakePatrol(COURSES[0], 0, 'external') if args.patrol else None, 'last': None, 'status': None}

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
            elif kind == 'drive' and patrol is None:
                drive.command(cid, msg.get('lin'), msg.get('ang'), time.monotonic())
            elif kind == 'patrol_start':
                course = next((c for c in COURSES if c['id'] == msg.get('course')), None)
                if patrol is not None:
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
                    plan = plan_course(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], map_data, pose[:2],
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
        web.send_to(cid, map_msg)

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
            manual['blocked'] = blocked
            rx, ry, ryaw = pose
            web.broadcast(build_state(pose, fake_scan(rx, ry, ryaw, people, 2), 7.38, -54, status,
                                      board.snapshot(t), True, t, manual=manual, patrol_running=running,
                                      patrol_app=patrol_app))
        time.sleep(dt)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n종료')
