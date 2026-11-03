"""7단계 통합 시험 채점: 시험 중 기록한 감지·알림을 실제 물체 위치와 맞춰 숫자로 정리한다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_eval_metrics.py).

물체 하나의 실제 위치는 '생성 위치(점)'이고, 걷기 시작한 뒤에는 '걷는 구간(선분)'도 더한다.
감지나 알림 위치가 그 점/선분에서 match_dist 안이면 그 물체로 본다. 어느 물체에도 안 맞으면 오경보다.
"""

import math


def distance_to_segment(px, py, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


class TruthObject:
    """시험에 넣은 물체 하나. spawned_at, walk 는 시험 중에 채워진다 (시뮬레이션 시각 [초])."""

    def __init__(self, name, x, y, person=False):
        self.name = name
        self.x, self.y = x, y
        self.person = person
        self.spawned_at = None
        self.walk = None             # (시작 시각, 시작점, 끝점)

    def distance(self, x, y, t):
        """시각 t 에 (x, y) 가 이 물체에서 얼마나 떨어져 있는지. 아직 없으면 무한대."""
        if self.spawned_at is None or t < self.spawned_at:
            return math.inf
        d = math.hypot(x - self.x, y - self.y)
        if self.walk is not None and t >= self.walk[0]:
            d = min(d, distance_to_segment(x, y, self.walk[1], self.walk[2]))
        return d


def match_object(objects, x, y, t, match_dist):
    """(x, y, t) 에 해당하는 물체를 돌려준다. 없으면 None."""
    best = min(objects, key=lambda o: o.distance(x, y, t), default=None)
    if best is None or best.distance(x, y, t) > match_dist:
        return None
    return best


def summarize(objects, detections, alerts, match_dist=0.6):
    """기록을 채점한다.

    detections: [{'event', 'x', 'y', 'stamp', 'first_seen', 'size', 'person', 'robot': (x, y) | None}, ...]
    alerts: [{'alert', 'incidents': [...], 'stamp'}, ...]
    돌려주는 값: {'objects': {이름: {...}}, 'false_alarms': [...], 'alert_counts': {...}}
    """
    result = {o.name: {'first_delay': None, 'confirm_delay': [], 'pos_error': [], 'robot_dist': None,
                       'sizes': set(), 'person': False, 'moving': False, 'incidents': set(),
                       'alerts': []} for o in objects}
    false_alarms = []
    for d in detections:
        if d['event'] not in ('detected', 'moving', 'person', 'upgraded'):
            continue
        obj = match_object(objects, d['x'], d['y'], d['stamp'], match_dist)
        if obj is None:
            if d['event'] == 'detected':
                false_alarms.append(d)
            continue
        r = result[obj.name]
        r['sizes'].add(d['size'])
        r['person'] = r['person'] or bool(d.get('person'))
        r['moving'] = r['moving'] or d['event'] == 'moving'
        if d['event'] != 'detected':
            continue
        if r['first_delay'] is None:
            r['first_delay'] = d['stamp'] - obj.spawned_at
            if d.get('robot'):
                r['robot_dist'] = math.hypot(d['robot'][0] - d['x'], d['robot'][1] - d['y'])
        if d.get('first_seen') is not None:
            r['confirm_delay'].append(d['stamp'] - d['first_seen'])
        if obj.walk is None or d['stamp'] < obj.walk[0]:
            r['pos_error'].append(math.hypot(d['x'] - obj.x, d['y'] - obj.y))

    counts = {}
    for a in alerts:
        counts[a['alert']] = counts.get(a['alert'], 0) + 1
        for i in a['incidents']:
            obj = match_object(objects, i['x'], i['y'], a['stamp'], match_dist + 0.2)
            if obj is None:
                continue
            r = result[obj.name]
            r['incidents'].add(i['incident'])
            r['alerts'].append((a['alert'], i['level'], bool(i.get('person'))))
    return {'objects': result, 'false_alarms': false_alarms, 'alert_counts': counts}


def report_lines(objects, summary, lap_times):
    """사람이 읽을 결과표 (터미널 출력과 파일 저장에 같이 쓴다)."""
    lines = ['# 7단계 통합 시험 결과', '']
    if lap_times:
        lines.append('바퀴 시간: ' + ', '.join(f'{i + 1}바퀴 {t:.0f}초' for i, t in enumerate(lap_times)))
    lines += ['', '| 물체 | 생성→첫 감지 | 확정 지연 | 위치 오차(최대) | 감지 거리 | 크기 | 사람 후보 | 이동 | 사건 수 | 알림 |',
              '|---|---|---|---|---|---|---|---|---|---|']
    for o in objects:
        r = summary['objects'][o.name]
        if o.spawned_at is None:
            lines.append(f'| {o.name} | 생성 안 됨 | | | | | | | | |')
            continue
        fmt = lambda v, unit: '-' if v is None else f'{v:.1f}{unit}'
        confirm = max(r['confirm_delay']) if r['confirm_delay'] else None
        error = max(r['pos_error']) if r['pos_error'] else None
        person = '예' if r['person'] else '아니오'
        if r['person'] != o.person:
            person += ' (기대와 다름)'
        alerts = ', '.join(f'{k}/{lv}{"/P" if p else ""}' for k, lv, p in r['alerts']) or '-'
        lines.append(
            f'| {o.name} | {fmt(r["first_delay"], "초")} | {fmt(confirm, "초")} | '
            f'{"-" if error is None else f"{error:.2f} m"} | {fmt(r["robot_dist"], " m")} | '
            f'{"/".join(sorted(r["sizes"])) or "-"} | {person} | {"예" if r["moving"] else "아니오"} | '
            f'{len(r["incidents"])} | {alerts} |')
    lines += ['', f'오경보(어느 물체에도 안 맞는 감지): {len(summary["false_alarms"])}건']
    for d in summary['false_alarms']:
        lines.append(f'  - ({d["x"]:+.2f}, {d["y"]:+.2f}) 크기 {d["size"]}, 시각 {d["stamp"]:.0f}')
    counts = summary['alert_counts']
    lines.append('알림 수: ' + (', '.join(f'{k} {v}' for k, v in sorted(counts.items())) or '없음'))
    return lines
