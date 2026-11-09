"""앱 화면 데이터 계산 시험 (ROS 불필요)."""

import base64
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from app_logic import ObjectBoard, build_state, encode_map, scan_to_points, yaw_from_quaternion  # noqa: E402


class AppLogicTest(unittest.TestCase):

    def test_yaw(self):
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, math.sin(0.5), math.cos(0.5)), 1.0)

    def test_encode_map_unknown_as_255(self):
        m = encode_map(3, 1, 0.05, -1.0, -2.0, 0.0, [-1, 0, 100])
        self.assertEqual(base64.b64decode(m['data']), bytes([255, 0, 100]))
        self.assertEqual((m['width'], m['height'], m['origin']), (3, 1, [-1.0, -2.0, 0.0]))

    def test_scan_front_and_left(self):
        # 4점: -180°, -90°, 0°, 90°
        pts = scan_to_points([1.0, 2.0, 3.0, 4.0], -math.pi, math.pi / 2, 0.15, 8.0, step=1)
        self.assertEqual(pts, [[-1.0, -0.0], [0.0, -2.0], [3.0, 0.0], [0.0, 4.0]])

    def test_scan_drops_invalid_and_steps(self):
        ranges = [0.0, 1.0, math.inf, 1.0, 9.0, 1.0, float('nan'), 1.0]
        pts = scan_to_points(ranges, 0.0, 0.1, 0.15, 8.0, step=1)
        self.assertEqual(len(pts), 4)
        self.assertEqual(len(scan_to_points([1.0] * 720, -math.pi, math.pi / 360, 0.15, 8.0, step=2)), 360)

    def test_scan_laser_offset_and_rotation(self):
        # 라이다가 로봇 앞 0.06 m 에 뒤를 보고 달려 있으면, 정면 1 m 점은 로봇 뒤 0.94 m
        pts = scan_to_points([1.0], 0.0, 0.1, 0.15, 8.0, laser_x=0.06, laser_yaw=math.pi, step=1)
        self.assertAlmostEqual(pts[0][0], -0.94)

    def test_object_board_keeps_cleared_for_a_while(self):
        b = ObjectBoard(keep_sec=30)
        b.update({'event': 'detected', 'id': 3, 'x': 1.234, 'y': 2.0, 'moving': False, 'person': False}, 0.0)
        b.update({'event': 'moving', 'id': 3, 'x': 1.5, 'y': 2.0, 'moving': True}, 1.0)
        snap = b.snapshot(1.0)
        self.assertEqual(snap[0]['x'], 1.5)
        self.assertTrue(snap[0]['visible'] and snap[0]['moving'])
        b.update({'event': 'cleared', 'id': 3, 'x': 1.6, 'y': 2.0}, 2.0)
        self.assertFalse(b.snapshot(20.0)[0]['visible'])
        self.assertEqual(b.snapshot(40.0), [])

    def test_build_state_without_pose(self):
        s = build_state(None, [], None, None, None, [], False, 12.345)
        self.assertEqual((s['type'], s['robot'], s['localized'], s['t']), ('state', None, False, 12.35))


if __name__ == '__main__':
    unittest.main()
