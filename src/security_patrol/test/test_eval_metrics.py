"""7단계 채점 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from eval_metrics import TruthObject, distance_to_segment, report_lines, summarize  # noqa: E402


def det(event, x, y, stamp, size='medium', person=False, first_seen=None, robot=None):
    return {'event': event, 'x': x, 'y': y, 'stamp': stamp, 'size': size, 'person': person,
            'first_seen': first_seen, 'robot': robot}


def objects():
    a = TruthObject('intruder1', -1.0, 0.0, person=True)
    b = TruthObject('test_box', 2.0, 0.0)
    a.spawned_at = b.spawned_at = 100.0
    return [a, b]


class EvalMetricsTest(unittest.TestCase):

    def test_segment_distance(self):
        self.assertAlmostEqual(distance_to_segment(0.0, 1.0, (-1, 0), (1, 0)), 1.0)
        self.assertAlmostEqual(distance_to_segment(2.0, 0.0, (-1, 0), (1, 0)), 1.0)

    def test_object_not_there_before_spawn(self):
        o = objects()[0]
        self.assertEqual(o.distance(-1.0, 0.0, 50.0), math.inf)

    def test_detection_metrics(self):
        objs = objects()
        s = summarize(objs, [det('detected', -1.04, 0.0, 103.0, person=True, first_seen=102.7, robot=(-3.0, 0.0))], [])
        r = s['objects']['intruder1']
        self.assertAlmostEqual(r['first_delay'], 3.0)
        self.assertAlmostEqual(r['confirm_delay'][0], 0.3)
        self.assertAlmostEqual(r['pos_error'][0], 0.04)
        self.assertAlmostEqual(r['robot_dist'], 1.96)
        self.assertTrue(r['person'])
        self.assertEqual(s['false_alarms'], [])

    def test_false_alarm_counted(self):
        s = summarize(objects(), [det('detected', 0.0, 2.0, 103.0)], [])
        self.assertEqual(len(s['false_alarms']), 1)

    def test_detection_before_spawn_is_false_alarm(self):
        s = summarize(objects(), [det('detected', -1.0, 0.0, 50.0)], [])
        self.assertEqual(len(s['false_alarms']), 1)

    def test_walking_object_matches_along_path(self):
        objs = objects()
        objs[0].walk = (200.0, (-1.0, 0.3), (0.4, 0.3))
        s = summarize(objs, [det('detected', 0.3, 0.35, 210.0), det('moving', 0.3, 0.35, 211.0)], [])
        r = s['objects']['intruder1']
        self.assertTrue(r['moving'])
        self.assertEqual(r['pos_error'], [])   # 걷는 중 위치는 오차 계산에서 뺀다
        self.assertEqual(s['false_alarms'], [])

    def test_alerts_grouped_per_object(self):
        alerts = [
            {'alert': 'new', 'stamp': 103.0, 'incidents': [{'incident': 1, 'level': '주의', 'x': -1.0, 'y': 0.0, 'person': True}]},
            {'alert': 'new', 'stamp': 104.0, 'incidents': [{'incident': 2, 'level': '주의', 'x': 2.0, 'y': 0.0}]},
            {'alert': 'reminder', 'stamp': 403.0, 'incidents': [
                {'incident': 1, 'level': '주의', 'x': -1.0, 'y': 0.0}, {'incident': 2, 'level': '주의', 'x': 2.0, 'y': 0.0}]},
        ]
        s = summarize(objects(), [], alerts)
        self.assertEqual(s['objects']['intruder1']['incidents'], {1})
        self.assertEqual(len(s['objects']['test_box']['alerts']), 2)
        self.assertEqual(s['alert_counts'], {'new': 2, 'reminder': 1})

    def test_report_flags_unexpected_person(self):
        objs = objects()
        s = summarize(objs, [det('detected', 2.0, 0.0, 103.0, person=True)], [])
        text = '\n'.join(report_lines(objs, s, [270.0]))
        self.assertIn('기대와 다름', text)
        self.assertIn('1바퀴 270초', text)

    def test_report_marks_unspawned(self):
        objs = objects()
        objs[1].spawned_at = None
        text = '\n'.join(report_lines(objs, summarize(objs, [], []), []))
        self.assertIn('생성 안 됨', text)


if __name__ == '__main__':
    unittest.main()
