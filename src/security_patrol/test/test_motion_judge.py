"""이동 판정 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from motion_judge import MotionJudge  # noqa: E402

DT = 1.0 / 6.0   # 스캔 주기 약 0.17초


def first_moving_time(positions, width=0.37, robot_speed=0.0):
    """positions(t) 를 8초 동안 넣고 처음 이동으로 판단한 시각을 돌려준다 (없으면 None)."""
    judge = MotionJudge()
    for k in range(48):
        t = k * DT
        x, y = positions(t)
        if judge.update(t, x, y, width, robot_speed):
            return t
    return None


class MotionJudgeTest(unittest.TestCase):

    def test_walking_person_detected_fast(self):
        t = first_moving_time(lambda t: (t * 1.0, 0.0))
        self.assertIsNotNone(t)
        self.assertLessEqual(t, 1.2)

    def test_slow_walk_detected(self):
        self.assertIsNotNone(first_moving_time(lambda t: (t * 0.3, 0.0)))

    def test_walk_detected_while_robot_drives(self):
        self.assertIsNotNone(first_moving_time(lambda t: (t * 1.0, 0.0), robot_speed=0.15))

    def test_standing_with_noise_not_moving(self):
        rng = random.Random(1)
        for _ in range(200):
            noise = {}

            def jitter(t):
                # 위치 추정 흔들림 ±10 cm
                return noise.setdefault(t, (rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1)))
            self.assertIsNone(first_moving_time(jitter))

    def test_hidden_leg_flip_not_moving(self):
        # 다리 하나가 가려졌다 보였다 하면 중심이 12.5 cm 튄다
        self.assertIsNone(first_moving_time(lambda t: (0.0, 0.125 if int(t) % 2 else 0.0)))

    def test_viewpoint_shift_while_robot_drives_not_moving(self):
        # 로봇이 지나가는 동안 폭 0.8 m 물체의 보이는 면이 한쪽 끝에서 반대쪽 끝으로 옮겨 감
        def shift(t):
            return (0.4 * math.tanh((t - 4.0) / 1.0), 0.0)
        self.assertIsNone(first_moving_time(shift, width=0.8, robot_speed=0.15))

    def test_fast_robot_defers(self):
        self.assertIsNone(first_moving_time(lambda t: (t * 1.0, 0.0), robot_speed=0.3))

    def test_back_and_forth_not_moving(self):
        # 제자리에서 왔다 갔다 하면 같은 방향이 아니다
        self.assertIsNone(first_moving_time(lambda t: (0.14 * math.sin(t * 6.0), 0.0)))

    def test_gap_restarts_history(self):
        judge = MotionJudge()
        for k in range(4):
            self.assertFalse(judge.update(k * DT, 0.0, 0.0, 0.2, 0.0))
        # 감지가 1초 쉰 사이에 위치 추정이 튀어도 이동으로 보지 않는다
        for k in range(3):
            self.assertFalse(judge.update(2.0 + k * DT, 1.0, 0.0, 0.2, 0.0))


if __name__ == '__main__':
    unittest.main()
