"""사건 관리 규칙 시험 (ROS 불필요): python3 -m unittest discover -s src/security_patrol/test"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'security_patrol'))
from incident_logic import IncidentBook  # noqa: E402


def event(kind, track, x=0.0, y=0.0, size='medium', width=0.3, moving=False):
    return {'event': kind, 'id': track, 'x': x, 'y': y, 'size': size, 'width': width, 'moving': moving}


class IncidentBookTest(unittest.TestCase):

    def setUp(self):
        self.book = IncidentBook()

    def test_new_medium_alerts_as_caution(self):
        alerts = self.book.on_event(event('detected', 1), 0.0)
        self.assertEqual(alerts[0]['alert'], 'new')
        self.assertEqual(alerts[0]['incidents'][0]['level'], '주의')

    def test_next_lap_same_place_is_same_incident(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.on_event(event('cleared', 1), 10.0)
        alerts = self.book.on_event(event('detected', 2, x=0.3), 250.0)
        self.assertEqual(alerts, [])
        self.assertEqual(len(self.book.incidents), 1)

    def test_other_place_is_new_incident(self):
        self.book.on_event(event('detected', 1), 0.0)
        alerts = self.book.on_event(event('detected', 2, x=2.0), 1.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 2)

    def test_static_to_moving_escalates_once(self):
        self.book.on_event(event('detected', 1), 0.0)
        alerts = self.book.on_event(event('moving', 1, x=0.5), 5.0)
        self.assertEqual(alerts[0]['alert'], 'escalated')
        self.assertEqual(alerts[0]['incidents'][0]['level'], '긴급')
        self.assertEqual(self.book.on_event(event('moving', 1, x=1.0), 6.0), [])

    def test_acked_object_that_moves_alerts_again(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.acknowledge(1, 1.0)
        alerts = self.book.on_event(event('moving', 1, x=0.5), 5.0)
        self.assertEqual(alerts[0]['incidents'][0]['acked'], False)

    def test_moving_target_lost_briefly_is_same_incident(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.on_event(event('moving', 1, x=1.0), 1.0)
        self.book.on_event(event('cleared', 1, x=2.0), 2.0)
        self.assertEqual(self.book.on_event(event('detected', 2, x=4.0), 5.0), [])
        self.assertEqual(len(self.book.incidents), 1)

    def test_moving_target_long_gone_is_new_incident(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.on_event(event('moving', 1, x=1.0), 1.0)
        self.book.on_event(event('cleared', 1, x=2.0), 2.0)
        self.assertEqual(len(self.book.on_event(event('detected', 2, x=4.0), 30.0)), 1)

    def test_small_waits_for_lap_change(self):
        self.book.on_lap(1, 0.0)
        self.assertEqual(self.book.on_event(event('detected', 1, size='small', width=0.1), 1.0), [])
        self.assertEqual(self.book.on_event(event('detected', 2, x=2.0, size='small', width=0.1), 2.0), [])
        self.assertEqual(self.book.on_lap(1, 3.0), [])
        alerts = self.book.on_lap(2, 4.0)
        self.assertEqual(alerts[0]['alert'], 'batch')
        self.assertEqual(len(alerts[0]['incidents']), 2)

    def test_small_sent_without_lap_signal(self):
        self.book.on_event(event('detected', 1, size='small', width=0.1), 0.0)
        self.assertEqual(self.book.tick(299.0), [])
        self.assertEqual(self.book.tick(300.0)[0]['alert'], 'batch')

    def test_small_upgraded_alerts_now_and_leaves_batch(self):
        self.book.on_event(event('detected', 1, size='small', width=0.1), 0.0)
        alerts = self.book.on_event(event('upgraded', 1, size='medium', width=0.3), 1.0)
        self.assertEqual(alerts[0]['alert'], 'new')
        self.assertEqual(self.book.pending_small, [])

    def test_upgraded_after_alert_stays_silent(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.assertEqual(self.book.on_event(event('upgraded', 1, size='large', width=0.7), 1.0), [])
        self.assertEqual(self.book.incidents[0].size, 'large')

    def test_small_moving_is_urgent_immediately(self):
        self.book.on_event(event('detected', 1, size='small', width=0.1), 0.0)
        alerts = self.book.on_event(event('moving', 1, x=0.5, size='small', width=0.1), 1.0)
        self.assertEqual(alerts[0]['incidents'][0]['level'], '긴급')
        self.assertEqual(self.book.pending_small, [])

    def test_reminder_until_ack(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.assertEqual(self.book.tick(299.0), [])
        self.assertEqual(self.book.tick(300.0)[0]['alert'], 'reminder')
        self.assertEqual(self.book.tick(301.0), [])
        self.assertEqual(self.book.tick(600.0)[0]['alert'], 'reminder')
        self.assertTrue(self.book.acknowledge(1, 601.0))
        self.assertEqual(self.book.tick(1000.0), [])

    def test_ack_unknown_incident(self):
        self.assertFalse(self.book.acknowledge(99, 0.0))

    def test_acked_same_place_stays_silent(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.acknowledge(1, 1.0)
        self.book.on_event(event('cleared', 1), 2.0)
        self.assertEqual(self.book.on_event(event('detected', 2), 250.0), [])


if __name__ == '__main__':
    unittest.main()
