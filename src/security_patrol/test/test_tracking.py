"""물체 짝짓기 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from tracking import associate  # noqa: E402


def track(x, y, confirmed=True, moving=False):
    return SimpleNamespace(x=x, y=y, confirmed=confirmed, moving=moving)


def cluster(x, y):
    return (x, y, 0.3, 10)


class AssociateTest(unittest.TestCase):

    def test_near_cluster_matches(self):
        t = track(0.0, 0.0)
        pairs, new, unmatched = associate([t], [cluster(0.2, 0.0)], 0.4, 1.0)
        self.assertEqual(pairs, [(t, cluster(0.2, 0.0), False)])
        self.assertEqual((new, unmatched), ([], []))

    def test_vm_jump_continues_moving_track(self):
        # 2026-11-03 VM 로그: 움직이는 #1 이 (-0.08, +0.18) 에 있는데 다음에 (-0.82, +0.24) 에서 잡혔다
        t = track(-0.08, 0.18, moving=True)
        pairs, new, unmatched = associate([t], [cluster(-0.82, 0.24)], 0.4, 1.0)
        self.assertEqual(pairs, [(t, cluster(-0.82, 0.24), True)])
        self.assertEqual(new, [])

    def test_jump_of_static_track_is_new_object(self):
        t = track(-0.08, 0.18, moving=False)
        pairs, new, unmatched = associate([t], [cluster(-0.82, 0.24)], 0.4, 1.0)
        self.assertEqual(pairs, [])
        self.assertEqual(new, [cluster(-0.82, 0.24)])
        self.assertEqual(unmatched, [t])

    def test_unconfirmed_mover_is_not_rematched(self):
        t = track(0.0, 0.0, confirmed=False, moving=True)
        pairs, new, _ = associate([t], [cluster(0.7, 0.0)], 0.4, 1.0)
        self.assertEqual(new, [cluster(0.7, 0.0)])

    def test_too_far_is_new_object(self):
        t = track(0.0, 0.0, moving=True)
        pairs, new, _ = associate([t], [cluster(1.5, 0.0)], 0.4, 1.0)
        self.assertEqual(new, [cluster(1.5, 0.0)])

    def test_mover_still_matched_nearby_is_not_stolen(self):
        # 움직이는 대상이 이번 스캔에 제자리에서 잡혔으면 1 m 떨어진 다른 물체는 새 물체다
        t = track(0.0, 0.0, moving=True)
        pairs, new, _ = associate([t], [cluster(0.1, 0.0), cluster(0.9, 0.0)], 0.4, 1.0)
        self.assertEqual(pairs, [(t, cluster(0.1, 0.0), False)])
        self.assertEqual(new, [cluster(0.9, 0.0)])


if __name__ == '__main__':
    unittest.main()
