"""앱 화면에 보낼 데이터를 만드는 계산 (ROS 없이 시험 가능: test/test_app_logic.py).

좌표 약속
  - 라이다 점은 로봇 기준(base_link: x 앞, y 왼쪽, m)으로 보낸다. 레이더 화면은 그대로 그리고,
    지도 화면은 로봇 위치(map 기준)로 옮겨 그린다. 그래서 위치추정이 없어도 레이더 화면은 보인다.
  - 지도는 OccupancyGrid 값(-1 모름, 0 빈칸 ~ 100 벽)을 바이트로 바꿔 base64 로 한 번만 보낸다.
"""

import base64
import math
import os


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


# ---------- 수동 주행 (2단계) ----------

ROBOT_HALF_LENGTH = 0.13   # 로봇 외곽 0.26 x 0.20 m (Nav2 footprint 와 같은 값)
ROBOT_HALF_WIDTH = 0.10


def clamp(value, low, high):
    return max(low, min(high, value))


class ManualDrive:
    """앱 조이스틱 수동 주행의 안전 규칙. 서버가 이 결과만 cmd_vel 로 보낸다.

    - 한 번에 한 화면만 조종한다. 먼저 움직인 화면이 주인이 되고, 정지하거나 연결이 끊기면 풀린다.
    - 앱 명령이 timeout 초 넘게 안 오면 멈춘다. (손을 뗐는데 정지 신호가 유실됐거나 Wi-Fi 가 끊긴 경우)
    - 앱은 -1~1 비율만 보내고, 실제 속도는 서버의 최대값으로 정한다. 앱이 보낸 값을 그대로 믿지 않는다.
    - 멈출 때는 0 속도를 stop_ticks 번 보내고 그 뒤에는 보내지 않는다. 계속 0 을 보내면
      자동 순찰(Nav2)이나 키보드 조종과 다투기 때문이다.
    """

    def __init__(self, max_linear=0.15, max_angular=1.0, timeout=0.5, stop_ticks=3):
        self.max_linear, self.max_angular = max_linear, max_angular
        self.timeout, self.stop_ticks = timeout, stop_ticks
        self.owner = None              # 조종 중인 앱 번호
        self.linear = self.angular = 0.0
        self.last = 0.0                # 마지막 명령 시각
        self.zero_left = 0             # 앞으로 보낼 0 속도 횟수
        self.reason = None             # 마지막으로 멈춘 이유 (화면 표시용)

    def command(self, cid, lin, ang, now):
        """앱 #cid 의 조이스틱 값(-1~1). 받아들이면 None, 아니면 거절 이유('busy')."""
        if self.owner not in (None, cid):
            return 'busy'
        try:
            lin, ang = float(lin), float(ang)
        except (TypeError, ValueError):
            return 'bad'
        if not (math.isfinite(lin) and math.isfinite(ang)):
            return 'bad'
        self.owner, self.last, self.reason = cid, now, None
        self.linear = clamp(lin, -1.0, 1.0) * self.max_linear
        self.angular = clamp(ang, -1.0, 1.0) * self.max_angular
        return None

    def stop(self, reason='stop'):
        """정지 (어느 화면이든 누를 수 있다)."""
        if self.owner is not None or self.linear or self.angular:
            self.zero_left = self.stop_ticks
            self.reason = reason
        self.owner = None
        self.linear = self.angular = 0.0

    def disconnect(self, cid):
        if self.owner == cid:
            self.stop('disconnected')

    def step(self, now):
        """주기마다 부른다. 보낼 (linear, angular) 또는 보낼 것이 없으면 None."""
        if self.owner is not None and now - self.last > self.timeout:
            self.stop('timeout')
        if self.owner is not None:
            return self.linear, self.angular
        if self.zero_left > 0:
            self.zero_left -= 1
            return 0.0, 0.0
        return None

    def snapshot(self):
        return {'owner': self.owner, 'linear': round(self.linear, 3), 'angular': round(self.angular, 3),
                'max_linear': self.max_linear, 'max_angular': self.max_angular, 'reason': self.reason}


def guard_speed(linear, points, gap=0.15, side_margin=0.04):
    """가는 방향(앞/뒤)으로 차체 폭 안에 gap m 보다 가까운 라이다 점이 있으면 전후진을 0 으로 막는다.

    회전은 막지 않는다 (제자리에서 돌아 빠져나갈 수 있게). 돌려주는 값: (linear, 'front' | 'back' | None)
    points 는 로봇 기준 점 [[x, y], ...]. 차체 안쪽 점(라이다가 자기 몸을 본 것)은 무시한다.
    gap 이 0 이하면 끈다.
    """
    if gap <= 0 or linear == 0:
        return linear, None
    half_w = ROBOT_HALF_WIDTH + side_margin
    for x, y in points:
        if abs(y) > half_w:
            continue
        if linear > 0 and ROBOT_HALF_LENGTH <= x < ROBOT_HALF_LENGTH + gap:
            return 0.0, 'front'
        if linear < 0 and -ROBOT_HALF_LENGTH - gap < x <= -ROBOT_HALF_LENGTH:
            return 0.0, 'back'
    return linear, None


# ---------- 코스 (3단계) ----------

def load_course(path):
    """순찰 경로 파일(patrol_route.yaml 형식) → 앱용 코스. 형식이 아니면 None.

    patrol_node 의 load_route 와 같은 파일을 읽지만, 앱에는 이름·좌표만 있으면 된다.
    """
    import yaml   # 맥(시험용)에는 없을 수 있어 여기서 읽는다

    try:
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        patrol = cfg['patrol']
        points = [{'name': str(p['name']), 'x': round(float(p['x']), 3), 'y': round(float(p['y']), 3)}
                  for p in patrol['checkpoints']]
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError):
        return None
    if not points:
        return None
    stem = os.path.splitext(os.path.basename(path))[0]
    return {'name': str(patrol.get('name') or stem.replace('_', ' ')),
            'dwell': float(patrol.get('dwell_time', 2.0)), 'points': points}


def list_courses(dirs):
    """폴더들에서 코스 파일을 찾는다. ({id: 경로}, [앱에 보낼 코스...]) 를 돌려준다.

    id 는 파일 이름(확장자 뺀 것)이고, 이름이 겹치면 뒤에 번호를 붙인다. 앱에는 경로를 보내지 않는다.
    """
    paths, courses = {}, []
    for d in dirs:
        d = os.path.expanduser(d)
        if not os.path.isdir(d):
            continue
        for fname in sorted(os.listdir(d)):
            if not fname.endswith(('.yaml', '.yml')):
                continue
            path = os.path.join(d, fname)
            course = load_course(path)
            if course is None:
                continue
            cid, n = os.path.splitext(fname)[0], 2
            while cid in paths:
                cid, n = f'{os.path.splitext(fname)[0]}_{n}', n + 1
            paths[cid] = path
            courses.append(dict(course, id=cid))
    return paths, courses


def build_state(robot_pose, points, battery, rssi, patrol, objects, localized, now,
                manual=None, patrol_running=None, patrol_app=None, mode=None):
    """앱에 주기적으로 보내는 상태 한 덩어리."""
    robot = None
    if robot_pose is not None:
        x, y, yaw = robot_pose
        robot = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
    return {'type': 'state', 't': round(now, 2), 'robot': robot, 'localized': localized,
            'scan': points, 'battery': None if battery is None else round(battery, 2),
            'rssi': rssi, 'patrol': patrol, 'patrol_running': patrol_running,
            'objects': objects, 'manual': manual, 'patrol_app': patrol_app, 'mode': mode}
