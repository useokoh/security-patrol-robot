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

    def test_vm_walker_log_stays_one_incident(self):
        # 2026-11-03 VM 6단계 시험 로그 순서 그대로. 수정 전에는 사건 3개로 갈라졌다.
        b = self.book
        self.assertEqual(b.on_event(event('detected', 1, x=-1.00, y=-0.02, width=0.35), 2998.9)[0]['alert'], 'new')
        b.acknowledge(1, 3016.8)
        b.on_event(event('cleared', 1, x=-0.97, y=0.09), 3095.1)
        b.on_event(event('detected', 5, x=-0.98, y=-0.09, width=0.25), 3514.7)
        b.on_event(event('cleared', 5, x=-0.98, y=0.18), 3540.0)
        # 침입자가 걷기 시작: 한쪽 다리만 보인 물체가 1 m 가까이 떨어진 곳에 잡힌다
        self.assertEqual(b.on_event(event('detected', 7, x=-0.08, y=0.44, size='small', width=0.10), 3718.4), [])
        b.on_event(event('cleared', 7, x=-0.08, y=0.44, size='small', width=0.10), 3722.4)
        self.assertEqual(b.on_event(event('detected', 8, x=-0.05, y=0.52, size='small', width=0.07), 3724.0), [])
        alerts = b.on_event(event('moving', 8, x=0.14, y=0.52, size='small', width=0.07), 3725.1)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0]['alert'], 'escalated')
        summary = alerts[0]['incidents'][0]
        self.assertEqual((summary['incident'], summary['level'], summary['acked']), (1, '긴급', False))
        self.assertEqual(len(b.incidents), 1)
        self.assertEqual(b.pending_small, [])
        self.assertEqual(b.on_event(event('upgraded', 8, x=0.14, y=0.52, width=0.24), 3726.4), [])
        b.on_event(event('cleared', 8, x=-0.50, y=0.48), 3727.8)
        self.assertEqual(b.on_event(event('detected', 9, x=-0.52, y=0.37, width=0.33), 3728.7), [])
        self.assertEqual(b.on_event(event('moving', 9, x=0.12, y=0.45, width=0.33), 3730.1), [])
        b.acknowledge(1, 3733.2)
        # 확인한 뒤 놓쳤다 다시 보여도 같은 사건이다
        b.on_event(event('cleared', 9, x=-0.70, y=0.37), 3769.0)
        self.assertEqual(b.on_event(event('detected', 10, x=0.20, y=0.44, width=0.28), 3769.3), [])
        self.assertEqual(b.on_event(event('moving', 10, x=0.22, y=0.41, width=0.28), 3775.4), [])
        self.assertEqual(len(b.incidents), 1)
        self.assertEqual(b.tick(3775.4 + 400), [])   # 확인했으므로 재알림 없음

    def test_merged_incident_named_in_alert(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('cleared', 1), 10.0)
        b.on_event(event('detected', 2, x=1.0), 20.0)          # 중형이라 바로 '새 사건 #2' 가 나감
        alerts = b.on_event(event('moving', 2, x=1.5), 22.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 1)
        self.assertEqual(alerts[0]['incidents'][0]['merged'], [2])

    def test_far_new_mover_is_separate_incident(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('cleared', 1), 10.0)
        b.on_event(event('detected', 2, x=4.0, size='small', width=0.1), 20.0)
        alerts = b.on_event(event('moving', 2, x=4.5), 22.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 2)
        self.assertEqual(len(b.incidents), 2)

    def test_static_still_in_view_is_not_merged(self):
        # 정지 물체가 지금도 보이면 근처에서 움직이는 것은 다른 대상이다
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('detected', 2, x=1.0, size='small', width=0.1), 20.0)
        alerts = b.on_event(event('moving', 2, x=1.5), 22.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 2)

    def test_static_seen_after_new_one_appeared_is_not_merged(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('detected', 2, x=1.0, size='small', width=0.1), 20.0)
        b.on_event(event('cleared', 1), 21.0)                  # 새 물체가 나타난 뒤에도 정지 물체가 보였다
        alerts = b.on_event(event('moving', 2, x=1.5), 22.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 2)

    def test_old_new_incident_is_not_merged(self):
        # 새 사건이 생긴 지 오래 지나서 움직이면 처음부터 따로 있던 물체로 본다
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('cleared', 1), 10.0)
        b.on_event(event('detected', 2, x=1.0), 20.0)
        alerts = b.on_event(event('moving', 2, x=1.5), 60.0)
        self.assertEqual(alerts[0]['incidents'][0]['incident'], 2)

    def test_acked_moving_target_relinks_silently(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('moving', 1, x=1.0), 1.0)
        b.acknowledge(1, 2.0)
        b.on_event(event('cleared', 1, x=2.0), 3.0)
        self.assertEqual(b.on_event(event('detected', 2, x=4.0), 5.0), [])
        self.assertEqual(len(b.incidents), 1)

    def test_small_person_candidate_alerts_now(self):
        # 한쪽 다리가 가려 소형으로 보여도 사람 후보면 묶음으로 미루지 않는다
        b = self.book
        self.assertEqual(b.on_event(event('detected', 1, size='small', width=0.1), 0.0), [])
        alerts = b.on_event(event('person', 1, size='small', width=0.1), 1.0)
        self.assertEqual(alerts[0]['alert'], 'new')
        self.assertEqual(alerts[0]['incidents'][0]['level'], '주의')
        self.assertTrue(alerts[0]['incidents'][0]['person'])
        self.assertEqual(b.pending_small, [])

    def test_small_detected_as_person_skips_batch(self):
        e = dict(event('detected', 1, size='small', width=0.1), person=True)
        self.assertEqual(self.book.on_event(e, 0.0)[0]['alert'], 'new')

    def test_person_on_notified_incident_alerts_once_and_keeps_level(self):
        # 7단계 시험에서 첫 알림 뒤에 사람 후보가 된 경우: 한 번만 짧게 알린다
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        alerts = b.on_event(event('person', 1), 1.0)
        self.assertEqual(alerts[0]['alert'], 'person')
        self.assertTrue(alerts[0]['incidents'][0]['person'])
        self.assertEqual(alerts[0]['incidents'][0]['level'], '주의')
        self.assertEqual(b.on_event(event('person', 1), 2.0), [])
        # 다음 바퀴에 같은 자리에서 사람 후보로 다시 잡혀도 또 알리지 않는다
        b.on_event(event('cleared', 1), 10.0)
        self.assertEqual(b.on_event(dict(event('detected', 2), person=True), 250.0), [])

    def test_person_on_acked_incident_alerts_but_stays_acked(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.acknowledge(1, 1.0)
        alerts = b.on_event(event('person', 1), 2.0)
        self.assertEqual(alerts[0]['alert'], 'person')
        self.assertTrue(alerts[0]['incidents'][0]['acked'])
        self.assertEqual(b.tick(1000.0), [])     # 확인한 사건이라 재알림은 다시 시작하지 않는다

    def test_person_alert_delays_next_reminder(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('person', 1), 200.0)
        self.assertEqual(b.tick(300.0), [])
        self.assertEqual(b.tick(500.0)[0]['alert'], 'reminder')

    def test_person_before_first_alert_is_not_extra(self):
        # 묶음을 기다리던 소형이 사람 후보가 되면 '새 사건' 하나만 나간다
        b = self.book
        b.on_event(event('detected', 1, size='small', width=0.1), 0.0)
        alerts = b.on_event(event('person', 1, size='small', width=0.1), 1.0)
        self.assertEqual([a['alert'] for a in alerts], ['new'])

    def test_relinked_moving_person_escalates_with_one_alert(self):
        # 다음 바퀴에 같은 자리에서 사람 후보이면서 움직이는 것으로 잡히면 등급 상승 하나에 표시를 싣는다
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('cleared', 1), 10.0)
        alerts = b.on_event(dict(event('detected', 2, moving=True), person=True), 250.0)
        self.assertEqual([a['alert'] for a in alerts], ['escalated'])
        self.assertEqual(alerts[0]['incidents'][0]['level'], '긴급')
        self.assertTrue(alerts[0]['incidents'][0]['person'])

    def test_person_flag_survives_next_lap_and_merge(self):
        b = self.book
        b.on_event(event('detected', 1), 0.0)
        b.on_event(event('person', 1), 1.0)
        b.on_event(event('cleared', 1), 10.0)
        b.on_event(event('detected', 2, x=1.0, size='small', width=0.1), 20.0)
        alerts = b.on_event(event('moving', 2, x=1.5), 22.0)
        self.assertTrue(alerts[0]['incidents'][0]['person'])

    def test_acked_same_place_stays_silent(self):
        self.book.on_event(event('detected', 1), 0.0)
        self.book.acknowledge(1, 1.0)
        self.book.on_event(event('cleared', 1), 2.0)
        self.assertEqual(self.book.on_event(event('detected', 2), 250.0), [])


if __name__ == '__main__':
    unittest.main()
