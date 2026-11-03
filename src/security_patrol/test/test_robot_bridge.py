"""실제 로봇 데이터 변환 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from robot_bridge_logic import (SEGMENT_BYTES, SEGMENT_SIZE, OdomIntegrator, ScanAssembler,  # noqa: E402
                                parse_segment, telemetry_extras, yaw_to_quaternion)

TICKS = 7 * 298 * 4
RADIUS = 0.065 / 2


def segment(sec, nanosec, index, mm):
    """펌웨어와 같은 바이트 배열을 만든다. mm: 거리 180개 또는 하나의 값."""
    values = mm if isinstance(mm, list) else [mm] * SEGMENT_SIZE
    return struct.pack('<iIB', sec, nanosec, index) + struct.pack('<' + 'H' * SEGMENT_SIZE, *values)


def telemetry(t, fl, rl, fr, rr, gz=0, bat=7400, rssi=-50):
    sec = int(t)
    return [sec, int(round((t - sec) * 1e9)), fl, rl, fr, rr, gz, bat, rssi]


class SegmentTest(unittest.TestCase):

    def test_parse(self):
        data = segment(12, 500, 2, list(range(SEGMENT_SIZE)))
        self.assertEqual(len(data), SEGMENT_BYTES)
        stamp, index, ranges = parse_segment(data)
        self.assertEqual((stamp, index), ((12, 500), 2))
        self.assertAlmostEqual(ranges[179], 0.179)

    def test_parse_accepts_ros_list_of_ints(self):
        # rclpy 의 UInt8MultiArray.data 는 array 나 int 목록으로 온다
        self.assertIsNotNone(parse_segment(list(segment(1, 0, 0, 1000))))

    def test_rejects_wrong_length_and_index(self):
        self.assertIsNone(parse_segment(segment(1, 0, 0, 1000)[:-1]))
        self.assertIsNone(parse_segment(segment(1, 0, 4, 1000)))

    def test_four_segments_make_one_scan(self):
        a = ScanAssembler()
        for i in range(3):
            self.assertIsNone(a.add(segment(5, 0, i, 1000 + i)))
        scan = a.add(segment(5, 0, 3, 1003))
        self.assertEqual(len(scan), 720)
        self.assertAlmostEqual(scan[0], 1.0)
        self.assertAlmostEqual(scan[719], 1.003)

    def test_lost_segment_drops_scan_instead_of_mixing(self):
        a = ScanAssembler()
        a.add(segment(5, 0, 0, 1000))
        a.add(segment(5, 0, 1, 1000))          # 2, 3 번 조각이 빠짐
        self.assertIsNone(a.add(segment(6, 0, 0, 2000)))
        self.assertIsNone(a.add(segment(6, 0, 1, 2000)))
        self.assertIsNone(a.add(segment(6, 0, 2, 2000)))
        scan = a.add(segment(6, 0, 3, 2000))
        self.assertTrue(all(abs(r - 2.0) < 1e-9 for r in scan))
        self.assertEqual(a.dropped, 1)


class OdomTest(unittest.TestCase):

    def test_first_message_only_sets_time(self):
        o = OdomIntegrator()
        self.assertFalse(o.update(telemetry(1.00, 100, 100, 100, 100)))
        self.assertEqual((o.x, o.y), (0.0, 0.0))

    def test_straight_one_wheel_turn(self):
        o = OdomIntegrator()
        o.update(telemetry(1.00, 0, 0, 0, 0))
        for k in range(100):                    # 100번에 나눠 바퀴 한 바퀴
            self.assertTrue(o.update(telemetry(1.01 + k * 0.01, TICKS / 100, TICKS / 100,
                                               TICKS / 100, TICKS / 100)))
        self.assertAlmostEqual(o.x, 2 * math.pi * RADIUS, places=6)
        self.assertAlmostEqual(o.y, 0.0, places=9)
        self.assertAlmostEqual(o.linear, 2 * math.pi * RADIUS / 100 / 0.01, places=6)
        self.assertAlmostEqual(o.wheel_angles[0], 2 * math.pi, places=6)

    def test_spin_uses_gyro(self):
        # 제자리 회전: 왼쪽 뒤로, 오른쪽 앞으로. 이동 거리 0, 회전은 자이로 90°/s × 1초
        o = OdomIntegrator()
        o.update(telemetry(0.0, 0, 0, 0, 0))
        for k in range(1, 101):
            o.update(telemetry(k * 0.01, -50, -50, 50, 50, gz=int(90 * 131)))
        self.assertAlmostEqual(math.degrees(o.yaw), 90.0, places=6)
        self.assertAlmostEqual(math.hypot(o.x, o.y), 0.0, places=9)

    def test_gyro_ignored_when_wheels_idle(self):
        o = OdomIntegrator()
        o.update(telemetry(0.0, 0, 0, 0, 0))
        for k in range(1, 101):
            o.update(telemetry(k * 0.01, 1, 0, -1, 0, gz=200))   # 정지 중 자이로 드리프트
        self.assertEqual(o.yaw, 0.0)

    def test_reboot_time_reset_is_skipped(self):
        o = OdomIntegrator()
        o.update(telemetry(100.0, 0, 0, 0, 0))
        self.assertFalse(o.update(telemetry(0.5, 1000, 1000, 1000, 1000)))
        self.assertEqual(o.x, 0.0)
        self.assertTrue(o.update(telemetry(0.51, 0, 0, 0, 0)))

    def test_long_gap_does_not_spin_far(self):
        o = OdomIntegrator()
        o.update(telemetry(0.0, 0, 0, 0, 0))
        o.update(telemetry(5.0, -50, -50, 50, 50, gz=int(90 * 131)))   # 5초 끊긴 뒤
        self.assertAlmostEqual(math.degrees(o.yaw), 90 * 0.2, places=6)

    def test_heading_wraps(self):
        o = OdomIntegrator()
        o.update(telemetry(0.0, 0, 0, 0, 0))
        for k in range(1, 401):
            o.update(telemetry(k * 0.01, -50, -50, 50, 50, gz=int(90 * 131)))   # 360°
        self.assertLess(abs(o.yaw), 1e-6)

    def test_wrong_length_ignored(self):
        self.assertFalse(OdomIntegrator().update([1, 2, 3]))


class HelperTest(unittest.TestCase):

    def test_quaternion(self):
        x, y, z, w = yaw_to_quaternion(math.pi / 2)
        self.assertAlmostEqual(z, math.sqrt(0.5))
        self.assertAlmostEqual(w, math.sqrt(0.5))
        self.assertEqual((x, y), (0.0, 0.0))

    def test_extras(self):
        self.assertEqual(telemetry_extras(telemetry(0, 0, 0, 0, 0, bat=7412, rssi=-61)), (7.412, -61))


if __name__ == '__main__':
    unittest.main()
