"""앱 화면 데이터 계산 시험 (ROS 불필요)."""

import base64
import math
import os
import sys
import tempfile
import unittest

try:
    import yaml  # noqa: F401  (코스 파일 시험에만 필요. 맥 기본 파이썬에는 없을 수 있다)
except ImportError:
    yaml = None

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from app_logic import (ManualDrive, ObjectBoard, build_state, encode_map, guard_speed,  # noqa: E402
                       list_courses, scan_to_points, yaw_from_quaternion)


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


class ManualDriveTest(unittest.TestCase):

    def setUp(self):
        self.d = ManualDrive(max_linear=0.2, max_angular=1.0, timeout=0.5, stop_ticks=3)

    def test_scales_and_clamps_app_values(self):
        self.assertIsNone(self.d.command(1, 0.5, -3.0, 0.0))     # 앱이 범위 밖 값을 보내도 최대값으로 자른다
        self.assertEqual(self.d.step(0.1), (0.1, -1.0))

    def test_rejects_garbage(self):
        self.assertEqual(self.d.command(1, 'fast', 0, 0.0), 'bad')
        self.assertEqual(self.d.command(1, float('nan'), 0, 0.0), 'bad')
        self.assertIsNone(self.d.owner)
        self.assertIsNone(self.d.step(0.1))                       # 아무 명령도 안 나감

    def test_one_driver_at_a_time(self):
        self.d.command(1, 1.0, 0.0, 0.0)
        self.assertEqual(self.d.command(2, -1.0, 0.0, 0.1), 'busy')
        self.assertEqual(self.d.step(0.1), (0.2, 0.0))
        self.d.stop()                                             # 정지 후에는 다른 화면이 잡을 수 있다
        self.assertIsNone(self.d.command(2, -1.0, 0.0, 0.2))
        self.assertEqual(self.d.owner, 2)

    def test_timeout_stops_then_goes_quiet(self):
        self.d.command(1, 1.0, 0.0, 0.0)
        self.assertEqual(self.d.step(0.4), (0.2, 0.0))
        out = [self.d.step(0.6 + i * 0.1) for i in range(5)]     # 0.5초 넘게 명령 없음
        self.assertEqual(out, [(0.0, 0.0)] * 3 + [None, None])   # 0 을 3번 보내고 조용해진다
        self.assertEqual(self.d.reason, 'timeout')
        self.assertIsNone(self.d.owner)

    def test_disconnect_only_stops_own_driver(self):
        self.d.command(1, 1.0, 0.0, 0.0)
        self.d.disconnect(2)
        self.assertEqual(self.d.owner, 1)
        self.d.disconnect(1)
        self.assertEqual((self.d.owner, self.d.reason, self.d.step(0.1)), (None, 'disconnected', (0.0, 0.0)))

    def test_idle_stop_sends_nothing(self):
        self.d.stop()                                             # 움직이지 않을 때 정지를 눌러도 0 을 보내지 않는다
        self.assertIsNone(self.d.step(0.0))


class GuardTest(unittest.TestCase):

    def test_blocks_forward_only_when_front_is_close(self):
        pts = [[0.25, 0.0]]                                       # 차체 앞(0.13) 12 cm 앞에 물체
        self.assertEqual(guard_speed(0.1, pts), (0.0, 'front'))
        self.assertEqual(guard_speed(-0.1, pts), (-0.1, None))    # 뒤로는 갈 수 있다
        self.assertEqual(guard_speed(0.0, pts), (0.0, None))

    def test_blocks_backward(self):
        self.assertEqual(guard_speed(-0.1, [[-0.2, 0.05]]), (0.0, 'back'))

    def test_ignores_side_far_and_self_points(self):
        pts = [[0.2, 0.3],        # 옆으로 비켜 있음
               [0.5, 0.0],        # 충분히 멀다
               [0.05, 0.0]]       # 차체 안쪽 = 라이다가 자기 몸을 본 것
        self.assertEqual(guard_speed(0.1, pts), (0.1, None))

    def test_can_be_turned_off(self):
        self.assertEqual(guard_speed(0.1, [[0.2, 0.0]], gap=0.0), (0.1, None))


@unittest.skipIf(yaml is None, 'pyyaml 없음')
class CourseTest(unittest.TestCase):

    def write(self, folder, name, text):
        with open(os.path.join(folder, name), 'w', encoding='utf-8') as f:
            f.write(text)

    def test_lists_only_patrol_files(self):
        a, b = tempfile.mkdtemp(), tempfile.mkdtemp()
        self.write(a, 'patrol_route.yaml', 'patrol:\n  name: 기본 코스\n  dwell_time: 3\n  checkpoints:\n'
                   '    - {name: 출발점, x: -3.0, y: 0.0}\n    - {name: 북쪽, x: 0, y: 2.4}\n')
        self.write(a, 'mapping_route.yaml', 'auto_mapper:\n  ros__parameters: {}\n')   # 다른 형식
        self.write(a, 'broken.yaml', 'patrol: [\n')                                    # 깨진 파일
        self.write(a, 'notes.txt', 'patrol:\n')
        self.write(b, 'patrol_route.yaml', 'patrol:\n  checkpoints:\n    - {name: A, x: 1, y: 2}\n')
        paths, courses = list_courses([a, b, '/없는/폴더'])
        self.assertEqual([c['id'] for c in courses], ['patrol_route', 'patrol_route_2'])   # 이름이 겹치면 번호
        self.assertEqual(courses[0]['name'], '기본 코스')
        self.assertEqual(courses[0]['dwell'], 3.0)
        self.assertEqual(courses[0]['points'][1], {'name': '북쪽', 'x': 0.0, 'y': 2.4})
        self.assertEqual(courses[1]['name'], 'patrol route')                             # 이름이 없으면 파일 이름
        self.assertEqual(paths['patrol_route_2'], os.path.join(b, 'patrol_route.yaml'))
        self.assertNotIn('path', courses[0])                                              # 앱에는 경로를 안 보낸다

    def test_real_package_course(self):
        here = os.path.join(os.path.dirname(__file__), '..', 'config')
        _, courses = list_courses([here])
        self.assertEqual([c['id'] for c in courses], ['patrol_route'])
        self.assertEqual(len(courses[0]['points']), 9)


if __name__ == '__main__':
    unittest.main()
