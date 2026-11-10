"""자율 탐사 계산 시험 (ROS 불필요). 가짜 '정답' 지도를 라이다로 조금씩 밝혀 가며 끝까지 탐사되는지 본다."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from explore_logic import choose_goal, frontier_clusters, frontier_near, known_area  # noqa: E402

RES = 0.05


def two_rooms():
    """6 x 3 m, x=3 m 에 벽, 아래쪽에 0.8 m 문. 원점 (0, 0)."""
    w, h = 120, 60
    data = []
    for r in range(h):
        for c in range(w):
            x, y = (c + 0.5) * RES, (r + 0.5) * RES
            wall = c == 0 or r == 0 or c == w - 1 or r == h - 1 or (2.95 <= x <= 3.05 and y >= 0.9)
            data.append(100 if wall else 0)
    return w, h, data


def reveal(truth, known, w, h, x, y, rng=2.5):
    """(x, y) 에서 라이다로 보이는 칸을 known 에 옮긴다."""
    for k in range(360):
        a = math.radians(k)
        for s in range(1, int(rng / (RES * 0.5))):
            c, r = int((x + math.cos(a) * s * RES * 0.5) / RES), int((y + math.sin(a) * s * RES * 0.5) / RES)
            if not (0 <= c < w and 0 <= r < h):
                break
            i = r * w + c
            known[i] = truth[i]
            if truth[i] >= 65:
                break


class FrontierTest(unittest.TestCase):

    def test_clusters_and_small_ones_dropped(self):
        w, h = 10, 10
        data = [-1] * 100
        for r in range(10):
            for c in range(5):
                data[r * w + c] = 0                    # 왼쪽 절반만 앎 → 세로 경계 한 줄 (10칸)
        data[0 * w + 8] = 0                            # 외딴 빈칸 하나 (잡음)
        clusters = frontier_clusters(w, h, data, min_cells=3)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(len(clusters[0]), 10)

    def test_fully_known_map_is_done(self):
        w, h, truth = two_rooms()
        pick = choose_goal(w, h, RES, 0.0, 0.0, truth, (1.0, 1.5))
        self.assertIsNone(pick['goal'])
        self.assertEqual(pick['frontiers'], 0)


class GoalTest(unittest.TestCase):

    def setUp(self):
        self.w, self.h, self.truth = two_rooms()
        self.known = [-1] * (self.w * self.h)
        reveal(self.truth, self.known, self.w, self.h, 1.0, 1.5)

    def test_goal_is_safe_known_free_cell(self):
        pick = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, (1.0, 1.5))
        gx, gy = pick['goal']
        i = int(gy / RES) * self.w + int(gx / RES)
        self.assertEqual(self.known[i], 0)                       # 아는 빈칸에서 바라본다
        self.assertGreater(min(gx, gy, 6 - gx, 3 - gy), 0.3)    # 바깥 벽에서 떨어져 있다
        self.assertTrue(frontier_near(self.w, self.h, RES, 0.0, 0.0, self.known, (gx, gy), 1.0))

    def test_blacklist_skips_goal(self):
        first = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, (1.0, 1.5))
        second = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, (1.0, 1.5), blacklist=[first['goal']])
        self.assertNotEqual(first['goal'], second['goal'])

    def test_standing_at_viewpoint_tries_deeper_spot(self):
        first = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, (1.0, 1.5))
        # 지도를 더 밝히지 않은 채 그 자리에 서면 (여기서는 안 보이는 경계) 덩어리 쪽 다른 관찰 지점을 고른다
        again = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, first['goal'])
        self.assertEqual(again['abandon'], [])
        self.assertGreater(math.dist(again['goal'], first['goal']), 0.25)

    def test_no_ping_pong_between_two_spots(self):
        # 지도를 더 밝히지 않은 채 A ↔ B 를 오가면, 가 본 곳은 다시 고르지 않아 결국 포기한다
        robot, visited, blacklist, seen = (1.0, 1.5), [], [], []
        for _ in range(6):
            pick = choose_goal(self.w, self.h, RES, 0.0, 0.0, self.known, robot, blacklist, visited)
            blacklist += pick['abandon']
            if pick['goal'] is None:
                break
            seen.append(pick['goal'])
            robot = pick['goal']
            visited.append(robot)
        self.assertEqual(len(seen), len(set(seen)))              # 같은 목표를 두 번 고르지 않는다

    def test_max_radius_keeps_robot_near_home(self):
        # 왼쪽 방에서 시작, 범위 2 m 면 3 m 밖 오른쪽 방 경계는 고르지 않는다
        w, h, truth = two_rooms()
        known = list(truth)
        for r in range(h):                             # 오른쪽 방 안쪽 깊은 곳만 모름
            for c in range(w):
                if (c + 0.5) * RES > 4.5:
                    known[r * w + c] = -1
        near = choose_goal(w, h, RES, 0.0, 0.0, known, (1.0, 1.5), home=(1.0, 1.5), max_radius=2.0)
        far = choose_goal(w, h, RES, 0.0, 0.0, known, (1.0, 1.5), home=(1.0, 1.5), max_radius=0.0)
        self.assertIsNone(near['goal'])
        self.assertIsNotNone(far['goal'])

    def test_unobservable_frontier_is_abandoned(self):
        # 약 0.7 m 좁은 방 안쪽 오른편에 모르는 띠: 벽에서 0.3 m 떨어진 관찰 지점은 가운데뿐이라
        # 가운데에 서 있는데도 경계가 남으면 볼 수 없는 경계로 보고 포기한다
        w = h = 16
        data = []
        for r in range(h):
            for c in range(w):
                x = (c + 0.5) * RES
                edge = c == 0 or r == 0 or c == w - 1 or r == h - 1
                data.append(100 if edge else -1 if 0.55 <= x <= 0.6 else 0)
        pick = choose_goal(w, h, RES, 0.0, 0.0, data, (0.4, 0.4), robot_radius=0.17, goal_clearance=0.30)
        self.assertIsNone(pick['goal'])
        self.assertGreaterEqual(len(pick['abandon']), 1)


class ExploreRunTest(unittest.TestCase):

    def test_explores_both_rooms(self):
        w, h, truth = two_rooms()
        known = [-1] * (w * h)
        robot, blacklist, visited = (1.0, 1.5), [], []
        reveal(truth, known, w, h, *robot)
        for _ in range(30):
            pick = choose_goal(w, h, RES, 0.0, 0.0, known, robot, blacklist, visited)
            blacklist += pick['abandon']
            if pick['goal'] is None:
                break
            robot = pick['goal']
            visited.append(robot)
            reveal(truth, known, w, h, *robot)
        else:
            self.fail('30번 안에 탐사가 끝나지 않음')
        self.assertGreater(known_area(known, RES) / known_area(truth, RES), 0.95)   # 두 방 모두 밝혀짐


if __name__ == '__main__':
    unittest.main()
