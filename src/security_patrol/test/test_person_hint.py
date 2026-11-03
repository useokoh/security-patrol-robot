"""사람 후보 판정 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from person_hint import is_leg_pair  # noqa: E402


def seg(x, y, width, points=6):
    return (x, y, width, points)


class LegPairTest(unittest.TestCase):

    def test_sim_intruder_two_legs(self):
        # 시뮬레이션 침입자: 지름 12 cm 기둥 두 개, 25 cm 간격. 보이는 면은 폭 8~12 cm
        self.assertTrue(is_leg_pair([seg(-1.0, 0.125, 0.10), seg(-1.0, -0.125, 0.09)]))

    def test_one_leg_hidden_is_not_pair(self):
        self.assertFalse(is_leg_pair([seg(-1.0, 0.125, 0.10)]))

    def test_box_is_not_pair(self):
        self.assertFalse(is_leg_pair([seg(0.0, 0.0, 0.35, 20)]))

    def test_wide_pieces_are_not_legs(self):
        self.assertFalse(is_leg_pair([seg(0.0, 0.0, 0.30), seg(0.0, 0.40, 0.30)]))

    def test_gap_too_small_or_large(self):
        self.assertFalse(is_leg_pair([seg(0.0, 0.0, 0.08), seg(0.0, 0.10, 0.08)]))
        self.assertFalse(is_leg_pair([seg(0.0, 0.0, 0.08), seg(0.0, 0.60, 0.08)]))

    def test_single_point_noise_is_ignored(self):
        legs = [seg(0.0, 0.0, 0.08), seg(0.0, 0.25, 0.08), seg(0.3, 0.1, 0.0, points=1)]
        self.assertTrue(is_leg_pair(legs))

    def test_three_pieces_are_not_pair(self):
        self.assertFalse(is_leg_pair([seg(0.0, 0.0, 0.08), seg(0.0, 0.25, 0.08), seg(0.0, 0.45, 0.08)]))


if __name__ == '__main__':
    unittest.main()
