"""6단계 알림 문구와 확인 버튼을 만든다.

ROS 없이 동작하는 순수 로직이라 맥에서도 시험할 수 있다 (test/test_telegram_alert.py).
문구는 측정한 사실만 쓰고 침입자나 사람이라고 단정하지 않는다 (다리 두 개 패턴이면 '사람 후보'까지만).
"""

import math

HEADERS = {
    'new': '새 사건',
    'escalated': '등급 상승',
    'person': '사람 후보 추가',
    'reminder': '미확인 재알림',
    'batch': '작은 물체 묶음',
}
DESCRIPTIONS = {True: '움직이는 대상', False: '멈춰 있는 새 물체'}
ICONS = {'긴급': '🚨', '주의': '⚠️'}


def nearest_point(x, y, points):
    """가장 가까운 순찰 지점 이름을 돌려준다. points: [{'name', 'x', 'y'}, ...]"""
    if not points:
        return None
    return min(points, key=lambda p: math.hypot(p['x'] - x, p['y'] - y))['name']


def format_alert(alert, points, session):
    """incident_manager 의 알림(dict)을 (문구, 버튼 줄 목록) 으로 바꾼다."""
    incidents = alert['incidents']
    top = '긴급' if any(i['level'] == '긴급' for i in incidents) else '주의'
    lines = [f'{ICONS[top]} [{top}] {HEADERS[alert["alert"]]}']
    keyboard = []
    for i in incidents:
        place = nearest_point(i['x'], i['y'], points)
        where = f'({i["x"]:+.2f}, {i["y"]:+.2f})'
        if place:
            where += f', {place} 근처'
        what = DESCRIPTIONS[i['moving']]
        if i.get('person'):
            what += '(사람 후보)'   # 다리 두 개 패턴일 뿐이라 '후보'로만 쓴다
        line = f'사건 #{i["incident"]} {what}: {where}, 폭 {i["width"]:.2f} m'
        if i.get('merged'):
            # 먼저 다른 번호로 알렸던 같은 대상이면 관리자가 헷갈리지 않게 밝힌다
            line += f' (같은 대상: 사건 {", ".join(f"#{n}" for n in i["merged"])})'
        lines.append(line)
        keyboard.append([{'text': f'#{i["incident"]} 확인',
                          'callback_data': f'ack:{session}:{i["incident"]}'}])
    return '\n'.join(lines), keyboard


def parse_ack(data, session):
    """버튼 값에서 사건 번호를 꺼낸다. 형식이 다르거나 이전 실행의 버튼이면 None.

    incident_manager 를 다시 켜면 사건 번호가 1부터 다시 시작하므로,
    이전 실행의 버튼으로 엉뚱한 사건을 확인하지 않도록 실행 번호(session)를 비교한다.
    """
    parts = str(data).split(':')
    if len(parts) != 3 or parts[0] != 'ack' or parts[1] != str(session) or not parts[2].isdigit():
        return None
    return int(parts[2])


def remaining_keyboard(keyboard, pressed_data):
    """누른 버튼을 뺀 나머지 버튼 줄을 돌려준다."""
    rows = [[b for b in row if b.get('callback_data') != pressed_data] for row in keyboard]
    return [row for row in rows if row]
