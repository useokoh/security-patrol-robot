#!/usr/bin/env python3
"""맥에서 앱 화면 미리보기 (ROS·가상머신 없이). 가짜 방·로봇·라이다·사람을 만들어 앱 서버와 같은 데이터를 보낸다.

  python3 tools/app_preview.py            # 브라우저에서 http://localhost:8765
  python3 tools/app_preview.py --port 9000

화면을 고칠 때 가상머신을 켜지 않고 바로 확인하는 용도다. 앱이 보내는 명령은 터미널에 찍기만 한다.
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'src', 'security_patrol')
sys.path.insert(0, os.path.join(PKG, 'security_patrol'))
from app_logic import ObjectBoard, build_state, encode_map, scan_to_points  # noqa: E402
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
    return encode_map(MAP_W, MAP_H, RES, ORIGIN[0], ORIGIN[1], 0.0, data)


def dist_to_segment(px, py, a, b):
    (x1, y1), (x2, y2) = a, b
    ex, ey = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (x1 + t * ex), py - (y1 + t * ey))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8765)
    args = ap.parse_args()
    map_msg = make_map()
    web = WebServer(os.path.join(PKG, 'web'), host='127.0.0.1', port=args.port,
                    on_message=lambda cid, m: print(f'앱 #{cid} → {m}'),
                    on_connect=lambda cid: web.send_to(cid, map_msg)).start_in_thread()
    print(f'미리보기: http://localhost:{web.port}  (Ctrl+C 로 종료)')
    board = ObjectBoard()
    route = ['출발점', '북쪽', '동쪽', '남쪽']
    t0 = time.time()
    while True:
        t = time.time() - t0
        # 로봇: 방 안에서 타원을 그리며 순찰
        a = t * 0.25
        rx, ry, ryaw = -0.5 + 1.6 * math.cos(a), -0.2 + 1.1 * math.sin(a), a + math.pi / 2
        # 사람: 10초 뒤 들어와서 천천히 걸음 (다리 두 개)
        people = []
        if t > 10:
            px, py = -2.2 + 0.25 * ((t - 10) % 16), 1.2
            people = [(px, py + 0.1, 0.06), (px, py - 0.1, 0.06)]
            board.update({'event': 'moving' if t > 16 else 'detected', 'id': 1, 'x': px, 'y': py,
                          'width': 0.3, 'size': 'medium', 'moving': t > 16, 'person': t > 13}, t)
        ranges = []
        for i in range(720):
            ang = -math.pi + i * (2 * math.pi / 720)
            r = ray(rx, ry, ryaw + ang, WALLS, people)
            ranges.append(r if r <= 8.0 else math.inf)
        points = scan_to_points(ranges, -math.pi, 2 * math.pi / 720, 0.15, 8.0)
        idx = int(t / 8) % len(route)
        patrol = {'state': 'watching' if (t % 8) > 6 else 'moving', 'lap': int(t / 32) + 1, 'index': idx,
                  'name': route[idx]}
        web.broadcast(build_state((rx, ry, ryaw), points, 7.38, -54, patrol, board.snapshot(t), True, t))
        time.sleep(0.2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n종료')
