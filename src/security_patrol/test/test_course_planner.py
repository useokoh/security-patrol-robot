"""지도 → 순찰 코스 자동 만들기 시험 (ROS·numpy 불필요). 작은 가짜 지도를 그려서 확인한다."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from course_planner import GridMap, clearance_map, order_tour, plan_course  # noqa: E402

RES = 0.05


def room_map(w_m, h_m, walls=(), unknown_outside=True):
    """w_m x h_m 방(바깥 벽 두께 1칸). walls: 막힌 사각형 [(x0, y0, x1, y1) m]. 원점 (0, 0)."""
    w, h = int(round(w_m / RES)), int(round(h_m / RES))
    data = []
    for r in range(h):
        for c in range(w):
            x, y = (c + 0.5) * RES, (r + 0.5) * RES
            edge = c == 0 or r == 0 or c == w - 1 or r == h - 1
            inner = any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in walls)
            data.append(100 if edge or inner else 0)
    return w, h, data


def clearance_at(w, h, data, x, y):
    m = GridMap(w, h, RES, 0.0, 0.0, data)
    return clearance_map(m)[m.cell_of(x, y)]


class ClearanceTest(unittest.TestCase):

    def test_distance_to_nearest_wall(self):
        w, h, data = room_map(2.0, 2.0)
        self.assertAlmostEqual(clearance_at(w, h, data, 1.0, 1.0), 0.95, delta=0.06)   # 가운데: 벽까지 약 0.95 m
        self.assertAlmostEqual(clearance_at(w, h, data, 0.3, 1.0), 0.25, delta=0.06)


class TourTest(unittest.TestCase):

    def test_two_opt_untangles_square(self):
        pts = [(0, 0), (1, 1), (1, 0), (0, 1)]                 # 꼬인 순서로 줘도
        dist = [[math.dist(a, b) for b in pts] for a in pts]
        tour = order_tour(dist)
        length = sum(dist[tour[k]][tour[(k + 1) % 4]] for k in range(4))
        self.assertAlmostEqual(length, 4.0)                     # 네모 둘레로 편다


class PlanTest(unittest.TestCase):

    def check_points(self, w, h, data, plan, clearance=0.35):
        m = GridMap(w, h, RES, 0.0, 0.0, data)
        clear = clearance_map(m)
        for p in plan['points']:
            self.assertGreaterEqual(clear[m.cell_of(p['x'], p['y'])], clearance - 1e-6, p)

    def test_open_room_needs_few_points(self):
        w, h, data = room_map(4.0, 3.0)
        plan = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.5))
        self.assertLessEqual(len(plan['points']), 3)            # 트인 방은 지점이 적다
        self.assertGreaterEqual(plan['coverage'], 0.95)
        self.check_points(w, h, data, plan)

    def test_two_rooms_through_door(self):
        # 6 x 3 m, x=3 m 에 벽, 아래쪽에 0.8 m 문
        w, h, data = room_map(6.0, 3.0, walls=[(2.95, 0.9, 3.05, 3.0)])
        plan = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.5))
        xs = [p['x'] for p in plan['points']]
        self.assertTrue(any(x < 2.9 for x in xs) and any(x > 3.1 for x in xs))   # 두 방 모두에 지점
        self.check_points(w, h, data, plan)
        self.assertGreater(plan['length'], 0.0)

    def test_closed_room_is_skipped(self):
        # 벽에 문이 없으면 오른쪽 방은 갈 수 없으니 지점을 두지 않는다
        w, h, data = room_map(6.0, 3.0, walls=[(2.95, 0.0, 3.05, 3.0)])
        plan = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.5))
        self.assertTrue(all(p['x'] < 2.9 for p in plan['points']))
        self.assertAlmostEqual(plan['reachable_area'], 2.85 * 2.9, delta=0.3)   # 왼쪽 방 빈칸 넓이

    def test_smaller_view_range_means_more_points(self):
        w, h, data = room_map(6.0, 4.0)
        near = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.0), view_range=1.2)
        far = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.0), view_range=3.5)
        self.assertGreater(len(near['points']), len(far['points']))

    def test_names_and_start_near_robot(self):
        w, h, data = room_map(6.0, 4.0)
        plan = plan_course(w, h, RES, 0.0, 0.0, data, (1.0, 1.0), view_range=1.2)
        names = [p['name'] for p in plan['points']]
        self.assertEqual(len(names), len(set(names)))           # 이름이 겹치지 않는다
        first = plan['points'][0]
        others = [math.dist((1, 1), (p['x'], p['y'])) for p in plan['points']]
        self.assertAlmostEqual(math.dist((1, 1), (first['x'], first['y'])), min(others), delta=0.6)

    def test_robot_outside_map_is_error(self):
        w, h, data = room_map(3.0, 3.0)
        with self.assertRaises(ValueError):
            plan_course(w, h, RES, 0.0, 0.0, data, (10.0, 10.0))

    def test_robot_touching_wall_snaps_inside(self):
        w, h, data = room_map(3.0, 3.0)
        plan = plan_course(w, h, RES, 0.0, 0.0, data, (0.06, 1.5))   # 벽에 붙어 있어도
        self.assertTrue(plan['points'])

    def test_too_narrow_space_is_error(self):
        w, h, data = room_map(0.6, 0.6)
        with self.assertRaises(ValueError):
            plan_course(w, h, RES, 0.0, 0.0, data, (0.3, 0.3))


if __name__ == '__main__':
    unittest.main()
