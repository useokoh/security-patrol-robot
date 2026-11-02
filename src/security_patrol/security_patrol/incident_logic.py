"""5-2 사건 관리 규칙: 감지 이벤트를 '사건'으로 묶고 언제 알릴지 정한다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_incident_logic.py).

규칙:
  - 물체 하나 = 사건 하나. 등급은 긴급(움직이는 대상) / 주의(멈춰 있는 새 물체) 2단계.
  - 다음 바퀴에 same_place_dist 안에서 다시 보이면 같은 사건이다 (다시 알리지 않음).
  - 움직이던 대상을 잠깐 놓친 뒤 가까운 곳에서 다시 보이면 같은 사건이다.
  - 소형 주의는 순찰 한 바퀴마다 묶어서 알린다. 바퀴 신호가 없어도 batch_max_wait 뒤에는 알린다.
  - 다시 알리는 경우는 '정지 → 이동'과 'remind_after 동안 미확인' 두 가지뿐이다.
  - '사라짐'은 사건을 닫지 않는다. 로봇이 멀어져도 안 보이기 때문이다. 사건은 관리자 확인으로 닫는다.

모든 메서드는 보낼 알림 목록을 돌려준다: [{'alert': 종류, 'incidents': [사건 요약, ...]}, ...]
"""

import math

URGENT = '긴급'
CAUTION = '주의'


class Incident:

    def __init__(self, incident_id, x, y, size, width, now):
        self.id = incident_id
        self.x, self.y = x, y
        self.size = size
        self.width = width
        self.level = CAUTION
        self.moving = False
        self.acked = False
        self.notified = False       # 관리자에게 한 번이라도 알렸는지 (소형은 묶음 전까지 False)
        self.created = now
        self.last_alert = None
        self.last_seen = now
        self.tracks = set()         # 지금 이 사건으로 보고 있는 감지 노드의 물체 번호

    def summary(self):
        return {'incident': self.id, 'level': self.level, 'moving': self.moving, 'size': self.size,
                'x': round(self.x, 2), 'y': round(self.y, 2), 'width': round(self.width, 2),
                'acked': self.acked}


class IncidentBook:

    def __init__(self, same_place_dist=0.5, relink_sec=10.0, relink_dist=3.0,
                 remind_after=300.0, batch_max_wait=300.0):
        self.same_place_dist = same_place_dist   # 같은 자리로 보는 거리 [m]
        self.relink_sec = relink_sec             # 움직이던 대상을 놓친 뒤 이어 붙이는 시간 [초]
        self.relink_dist = relink_dist           # 움직이던 대상을 놓친 뒤 이어 붙이는 거리 [m]
        self.remind_after = remind_after         # 미확인 사건을 다시 알리는 간격 [초]
        self.batch_max_wait = batch_max_wait     # 소형 묶음을 최대로 기다리는 시간 [초]
        self.incidents = []
        self.by_track = {}                       # 감지 노드 물체 번호 → 사건
        self.pending_small = []                  # 다음 묶음 알림에 넣을 소형 사건
        self.next_id = 1
        self.lap = None

    # ---------- 감지 이벤트 ----------

    def on_event(self, event, now):
        """감지 노드 이벤트(dict) 하나를 처리한다."""
        kind = event['event']
        if kind == 'detected':
            return self._on_detected(event, now)
        incident = self.by_track.get(event['id'])
        if incident is None:
            return []
        if kind == 'moving':
            return self._on_moving(incident, event, now)
        if kind == 'upgraded':
            return self._on_upgraded(incident, event, now)
        if kind == 'cleared':
            incident.x, incident.y = event['x'], event['y']
            incident.last_seen = now
            incident.tracks.discard(event['id'])
            del self.by_track[event['id']]
        return []

    def _on_detected(self, event, now):
        incident = self._find_existing(event, now)
        if incident is None:
            incident = Incident(self.next_id, event['x'], event['y'], event['size'], event['width'], now)
            self.next_id += 1
            self.incidents.append(incident)
            self._link(incident, event, now)
            if event.get('moving'):
                return self._make_urgent(incident, now)
            if incident.size == 'small':
                self.pending_small.append(incident)
                return []
            return self._send('new', [incident], now)

        self._link(incident, event, now)
        incident.width = max(incident.width, event['width'])
        if event.get('moving') and not incident.moving:
            return self._make_urgent(incident, now)
        return []

    def _on_moving(self, incident, event, now):
        incident.x, incident.y = event['x'], event['y']
        incident.last_seen = now
        if incident.moving:
            return []
        return self._make_urgent(incident, now)

    def _on_upgraded(self, incident, event, now):
        incident.size = event['size']
        incident.width = max(incident.width, event['width'])
        incident.last_seen = now
        # 묶음을 기다리던 소형이 더 커졌으면 바로 알린다. 이미 알린 사건은 다시 알리지 않는다.
        if incident in self.pending_small:
            self.pending_small.remove(incident)
            return self._send('new', [incident], now)
        return []

    def _find_existing(self, event, now):
        """이미 있는 사건 중 이 감지와 같은 물체로 볼 사건을 찾는다."""
        best, best_dist = None, None
        for incident in self.incidents:
            dist = math.hypot(incident.x - event['x'], incident.y - event['y'])
            same_place = dist <= self.same_place_dist
            relink = (incident.moving and not incident.acked and not incident.tracks
                      and now - incident.last_seen <= self.relink_sec and dist <= self.relink_dist)
            if (same_place or relink) and (best is None or dist < best_dist):
                best, best_dist = incident, dist
        return best

    def _link(self, incident, event, now):
        incident.tracks.add(event['id'])
        self.by_track[event['id']] = incident
        incident.last_seen = now

    def _make_urgent(self, incident, now):
        incident.moving = True
        incident.level = URGENT
        incident.acked = False      # 확인한 물체가 움직이기 시작하면 새로 확인해야 한다
        if incident in self.pending_small:
            self.pending_small.remove(incident)
            return self._send('new', [incident], now)
        kind = 'escalated' if incident.notified else 'new'
        return self._send(kind, [incident], now)

    # ---------- 시간과 순찰 신호 ----------

    def on_lap(self, lap, now):
        """순찰 바퀴 번호를 받는다. 바퀴가 바뀌면 소형 묶음을 보낸다."""
        previous, self.lap = self.lap, lap
        if previous is None or lap == previous:
            return []
        return self._flush_small(now)

    def tick(self, now):
        """주기적으로 호출: 오래 기다린 소형 묶음과 미확인 사건 재알림을 처리한다."""
        alerts = []
        if self.pending_small and now - self.pending_small[0].created >= self.batch_max_wait:
            alerts += self._flush_small(now)
        due = [i for i in self.incidents
               if i.notified and not i.acked and now - i.last_alert >= self.remind_after]
        if due:
            alerts += self._send('reminder', due, now)
        return alerts

    def acknowledge(self, incident_id, now):
        """관리자 확인. 확인된 사건이 있으면 True."""
        for incident in self.incidents:
            if incident.id == incident_id:
                incident.acked = True
                if incident in self.pending_small:
                    self.pending_small.remove(incident)
                return True
        return False

    def _flush_small(self, now):
        if not self.pending_small:
            return []
        batch, self.pending_small = self.pending_small, []
        return self._send('batch', batch, now)

    def _send(self, kind, incidents, now):
        for incident in incidents:
            incident.notified = True
            incident.last_alert = now
        return [{'alert': kind, 'incidents': [i.summary() for i in incidents]}]
