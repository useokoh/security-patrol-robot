"""앱 화면에 보낼 데이터를 만드는 계산 (ROS 없이 시험 가능: test/test_app_logic.py).

좌표 약속
  - 라이다 점은 로봇 기준(base_link: x 앞, y 왼쪽, m)으로 보낸다. 레이더 화면은 그대로 그리고,
    지도 화면은 로봇 위치(map 기준)로 옮겨 그린다. 그래서 위치추정이 없어도 레이더 화면은 보인다.
  - 지도는 OccupancyGrid 값(-1 모름, 0 빈칸 ~ 100 벽)을 바이트로 바꿔 base64 로 한 번만 보낸다.
"""

import base64
import math


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def encode_map(width, height, resolution, origin_x, origin_y, origin_yaw, data):
    """OccupancyGrid → 앱용 dict. 모름(-1)은 255 로 바꾼다."""
    raw = bytes(255 if v < 0 else min(int(v), 100) for v in data)
    return {'type': 'map', 'width': width, 'height': height, 'resolution': round(resolution, 4),
            'origin': [round(origin_x, 3), round(origin_y, 3), round(origin_yaw, 4)],
            'data': base64.b64encode(raw).decode()}


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   laser_x=0.0, laser_y=0.0, laser_yaw=0.0, step=2, max_range=None):
    """LaserScan 거리값 → 로봇 기준 점 [[x, y], ...] (cm 단위로 반올림해 크기를 줄인다).

    step 개마다 하나씩만 쓴다 (720점 → 360점). 잘못된 값(0, inf, 범위 밖)은 버린다.
    laser_* 는 로봇(base_link) 기준 라이다 위치와 방향이다.
    """
    limit = range_max if max_range is None else min(range_max, max_range)
    points = []
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    for i in range(0, len(ranges), max(1, step)):
        r = ranges[i]
        if not (math.isfinite(r) and range_min <= r <= limit):
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        points.append([round(laser_x + c * lx - s * ly, 2), round(laser_y + s * lx + c * ly, 2)])
    return points


class ObjectBoard:
    """감지 노드 이벤트(anomaly/detections)로 지금 보이는 물체 목록을 관리한다.

    'cleared' 는 사건이 끝났다는 뜻이 아니라 감지 노드가 잠깐 못 보게 된 것이라,
    바로 지우지 않고 keep_sec 동안 흐리게 남긴다.
    """

    def __init__(self, keep_sec=30.0):
        self.keep_sec = keep_sec
        self.objects = {}

    def update(self, event, now):
        oid = event.get('id')
        if oid is None:
            return
        kind = event.get('event')
        obj = self.objects.setdefault(oid, {'id': oid})
        for key in ('x', 'y', 'size', 'width', 'moving', 'person'):
            if key in event:
                obj[key] = event[key]
        obj['visible'] = kind != 'cleared'
        obj['last'] = now

    def snapshot(self, now):
        for oid in [k for k, o in self.objects.items()
                    if not o['visible'] and now - o['last'] > self.keep_sec]:
            del self.objects[oid]
        return [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items() if k != 'last'}
                for o in self.objects.values()]


def build_state(robot_pose, points, battery, rssi, patrol, objects, localized, now):
    """앱에 주기적으로 보내는 상태 한 덩어리."""
    robot = None
    if robot_pose is not None:
        x, y, yaw = robot_pose
        robot = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
    return {'type': 'state', 't': round(now, 2), 'robot': robot, 'localized': localized,
            'scan': points, 'battery': None if battery is None else round(battery, 2),
            'rssi': rssi, 'patrol': patrol, 'objects': objects}
