"""앱 서버 노드: 로봇 상태를 모아 웹앱(휴대폰·맥 브라우저)에 보내고, 앱의 명령을 받는다.

1단계: 레이더 화면 - 지도, 로봇 위치, 라이다 점, 감지 물체, 순찰 상태, 배터리·Wi-Fi
2단계: 수동 주행 - 조이스틱 → cmd_vel. 손을 떼거나, 명령이 0.5초 끊기거나, 연결이 끊기면 정지.
       앞뒤가 막히면 전후진만 막고, 순찰이 켜져 있으면 수동 주행을 받지 않는다.
3단계: 코스 선택·자동 순찰 - 코스 파일 목록을 앱에 보내고, 앱이 고른 코스로 patrol_node 를 켜고 끈다.
3.5단계: 코스 자동 만들기 - 지금 지도와 로봇 위치로 순찰 지점을 뽑아(course_planner) 미리 보여 주고 저장한다.
3.5단계 ③: 모드 바꾸기 - app.launch.py 로 켜면(start_mode:=patrol) 앱 서버가 순찰 모드(mode_patrol.launch.py)와
          지도 만들기 모드(mode_explore.launch.py, 자율 탐사)를 켜고 끈다 (ModeRunner).

  # 보통은 app.launch.py 로 한 번에 켠다 (시뮬레이션/실제 로봇 + Nav2 + 감지 + 앱 서버)
  ros2 launch security_patrol app.launch.py
  # 앱 서버만 따로 (시뮬레이션)
  ros2 run security_patrol app_server --ros-args -p use_sim_time:=true
  # 앱 서버만 따로 (실제 로봇: 4륜이 미끄러져 회전이 명령의 약 25% 라 회전 상한을 올린다)
  ros2 run security_patrol app_server --ros-args -p max_angular:=1.8
  # 브라우저: 가상머신 안 http://localhost:8080, 맥 http://192.168.64.2:8080
"""

import datetime
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from security_patrol.app_logic import (ManualDrive, ObjectBoard, build_state, encode_map, guard_speed,
                                       list_courses, scan_to_points, yaw_from_quaternion)
from security_patrol.course_planner import plan_course
from security_patrol.mode_runner import ModeRunner
from security_patrol.patrol_runner import PatrolRunner
from security_patrol.route_file import route_yaml
from security_patrol.web_server import WebServer


class AppServer(Node):

    def __init__(self):
        super().__init__('app_server')
        p = self.declare_parameter
        port = int(p('port', 8080).value)
        web_dir = p('web_dir', os.path.join(get_package_share_directory('security_patrol'), 'web')).value
        rate = float(p('rate', 5.0).value)                       # 앱에 상태를 보내는 횟수 [Hz]
        self.scan_step = int(p('scan_step', 2).value)            # 라이다 점을 몇 개마다 하나씩 보낼지
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        # 수동 주행: 앱은 -1~1 비율만 보내고 실제 최대 속도는 여기서 정한다
        self.drive = ManualDrive(max_linear=float(p('max_linear', 0.15).value),      # [m/s]
                                 max_angular=float(p('max_angular', 1.0).value),     # [rad/s]
                                 timeout=float(p('drive_timeout', 0.5).value))       # 명령 끊김 허용 [초]
        self.guard_gap = float(p('guard_gap', 0.15).value)       # 차체 앞뒤 이 거리 안에 물체가 있으면 전후진 막음 [m], 0 = 끔
        drive_rate = float(p('drive_rate', 10.0).value)          # cmd_vel 보내는 횟수 [Hz]
        self.drive_lock = threading.Lock()                       # 앱 명령(웹 서버 스레드) ↔ 발행(ROS 스레드)
        self.blocked = None
        self.patrol_running = False
        self.exploring = False

        # 자동 순찰: 코스 파일을 찾을 폴더 (쉼표로 여러 개). 기본은 시뮬레이션이면 패키지 config,
        # 실제 로봇이면 저장소 밖 ~/maps_real (route_recorder 가 저장하는 곳)
        self.sim = bool(self.get_parameter('use_sim_time').value)
        share = get_package_share_directory('security_patrol')
        # 자동으로 만든 코스를 저장할 곳 (시뮬레이션 코스와 실제 공간 코스가 섞이지 않게 나눈다)
        self.save_dir = os.path.expanduser(p('save_dir', '~/sentinel_courses/sim' if self.sim else '~/maps_real').value)
        default_dirs = f'{os.path.join(share, "config")},{self.save_dir}' if self.sim else self.save_dir
        self.course_dirs = [d.strip() for d in str(p('course_dirs', default_dirs).value).split(',') if d.strip()]
        self.map_raw = None             # 코스 자동 만들기용 지도 원본
        self.robot_pose = None          # 마지막으로 구한 로봇 위치 (map 기준)
        self.plans = {}                 # 앱별 마지막 미리보기 코스
        self.planning = False
        self.course_paths, self.courses = {}, []
        self.runner = PatrolRunner(os.path.join(get_package_prefix('security_patrol'), 'lib', 'security_patrol',
                                                'patrol_node'), log=self.get_logger().info)
        self.refresh_courses()

        # 모드(순찰 / 지도 만들기): start_mode 가 비어 있으면 모드를 관리하지 않는다 (예전처럼 따로 켠 Nav2 를 씀)
        self.robot = str(p('robot', 'sim' if self.sim else 'real').value)
        self.start_mode = str(p('start_mode', '').value)
        self.detect = bool(p('detect', True).value)
        self.rviz = bool(p('rviz', False).value)
        self.default_map = (os.path.join(share, 'maps', 'restricted_area.yaml') if self.sim
                            else os.path.expanduser('~/maps_real/real_area.yaml'))
        self.map_file = os.path.expanduser(str(p('map_file', '').value)) or self.default_map
        self.explored_map = os.path.expanduser('~/sentinel_maps/sim/explored' if self.sim else '~/maps_real/explored_area')
        self.modes = ModeRunner(log=self.get_logger().info)
        self.mode_busy = False          # 모드를 바꾸는 중 (끄고 켜는 데 수십 초)
        self.explore = None             # /explore/status 마지막 값
        # 탐사 지도로 순찰 모드를 켤 때 AMCL 처음 위치를 '탐사가 끝난 위치'로 맞춘다 (출발점 복귀 실패 대비)
        self.pending_pose = None        # {'pose', 'deadline', 'last', 'sent_at'}
        self.amcl_pose = None           # (x, y, 받은 시각)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.scan = None
        self.scan_at = 0.0              # 라이다를 받은 시각 (time.monotonic)
        self.laser_pose = None          # 로봇 기준 라이다 위치 (x, y, yaw), 한 번 구하면 고정
        self.battery = None
        self.rssi = None
        self.patrol = None
        self.board = ObjectBoard()

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'patrol/status', self.on_patrol, 10)
        self.create_subscription(String, 'anomaly/detections', self.on_detection, 10)
        self.create_subscription(Float32, 'battery_voltage', lambda m: setattr(self, 'battery', m.data), 10)
        self.create_subscription(Int32, 'rssi_strength', lambda m: setattr(self, 'rssi', m.data), 10)
        self.create_subscription(String, 'explore/status', self.on_explore, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl_pose, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.web = WebServer(web_dir, port=port, on_message=self.on_app_message,
                             on_connect=self.on_app_connect, log=self.get_logger().info).start_in_thread()
        self.create_timer(1.0 / rate, self.tick)
        # 수동 주행 정지 판단은 실제 시간으로 한다 (시뮬레이션 시계가 멈추거나 느려도 끊김을 잡도록)
        self.create_timer(1.0 / drive_rate, self.drive_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'앱 서버 시작: http://<이 컴퓨터 주소>:{self.web.port} (화면 파일 {web_dir})')
        if self.start_mode in ('patrol', 'explore'):
            self.switch_mode_async(self.start_mode, self.map_file)

    # ---------- ROS 입력 ----------

    def on_map(self, msg):
        info = msg.info
        q = info.origin.orientation
        self.map_raw = (info.width, info.height, info.resolution,
                        info.origin.position.x, info.origin.position.y, list(msg.data))
        self.map_msg = encode_map(info.width, info.height, info.resolution,
                                  info.origin.position.x, info.origin.position.y,
                                  yaw_from_quaternion(q.x, q.y, q.z, q.w), msg.data)
        self.web.broadcast(self.map_msg)
        self.get_logger().info(f'지도 받음: {info.width}x{info.height}, 해상도 {info.resolution:.3f} m')

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.monotonic()

    def on_patrol(self, msg):
        try:
            self.patrol = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def on_explore(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        done_now = data.get('state') == 'done' and (self.explore or {}).get('state') != 'done'
        self.explore = data
        if done_now:                                   # 탐사로 만든 코스가 목록에 보이게
            self.get_logger().info(f'탐사 끝: 지도 {data.get("map_file")}, 코스 {data.get("course_file")}')
            self.refresh_courses()
            self.web.broadcast({'type': 'courses', 'courses': self.courses})

    def on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        self.amcl_pose = (p.x, p.y, time.monotonic())

    def send_initial_pose(self):
        """tick 에서 부른다. AMCL 이 켜져 받아들일 때까지 2초마다 보내고, AMCL 위치가 맞으면 멈춘다."""
        pend = self.pending_pose
        if pend is None:
            return
        now = time.monotonic()
        x, y, yaw = pend['pose']
        if pend['sent_at'] is not None and self.amcl_pose and self.amcl_pose[2] > pend['sent_at'] \
                and math.hypot(self.amcl_pose[0] - x, self.amcl_pose[1] - y) < 0.3:
            self.get_logger().info(f'위치추정을 탐사가 끝난 위치로 맞춤 ({x:+.2f}, {y:+.2f})')
            self.pending_pose = None
            return
        if now > pend['deadline']:
            self.get_logger().warn('위치추정 처음 위치를 맞추지 못함 (AMCL 응답 없음). RViz 2D Pose Estimate 로 맞추세요')
            self.pending_pose = None
            return
        if now - pend['last'] < 2.0 or self.count_subscribers('initialpose') == 0:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05                          # x, y 분산 [m²]
        cov[35] = 0.07                                  # yaw 분산 [rad²]
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)
        pend['last'] = now
        if pend['sent_at'] is None:
            pend['sent_at'] = now

    # ---------- 모드 (순찰 / 지도 만들기) ----------

    def mode_command(self, mode, map_file=None):
        cmd = ['ros2', 'launch', 'security_patrol', f'mode_{mode}.launch.py', f'robot:={self.robot}',
               f'rviz:={"true" if self.rviz else "false"}']
        if mode == 'patrol':
            cmd += [f'map:={map_file or self.default_map}', f'detect:={"true" if self.detect else "false"}']
        return cmd

    def switch_mode_async(self, mode, map_file=None):
        """모드 바꾸기는 끄고 켜는 데 시간이 걸리므로 따로 돌린다. 이미 바꾸는 중이면 False."""
        if self.mode_busy:
            return False
        self.mode_busy = True
        threading.Thread(target=self.switch_mode, args=(mode, map_file), daemon=True).start()
        return True

    def switch_mode(self, mode, map_file=None):
        log = self.get_logger()
        try:
            with self.drive_lock:                      # 바꾸는 동안 아무것도 로봇을 움직이지 않게
                self.drive.stop('mode')
            self.runner.stop(wait=True)
            # 탐사 모드에서 탐사 지도로 바꿀 때: 끄기 전 SLAM 기준 로봇 위치를 기억 (탐사 지도와 같은 좌표)
            from_explore = self.modes.snapshot()['mode'] == 'explore'
            last_pose = self.robot_pose
            self.modes.stop()
            # 지난 모드의 지도·물체·순찰 정보를 지운다 (새 모드가 새 지도를 보낸다)
            self.map_msg = self.map_raw = None
            self.board = ObjectBoard()
            self.patrol, self.explore = None, None
            if mode == 'patrol':
                self.map_file = map_file or self.default_map
            ok = self.modes.start(mode, self.mode_command(mode, self.map_file), {'map': self.map_file})
            if ok and mode == 'patrol' and from_explore and last_pose is not None \
                    and self.map_file == self.explored_map + '.yaml':
                self.pending_pose = {'pose': last_pose, 'deadline': time.monotonic() + 90.0, 'last': 0.0,
                                     'sent_at': None}
            text = ({'patrol': f'순찰 모드 (지도 {os.path.basename(self.map_file)})',
                     'explore': '지도 만들기 모드 (자율 탐사)'}[mode] + (' 시작' if ok else ' 실행 실패'))
            log.info(text)
            self.web.broadcast({'type': 'notice', 'level': 'ok' if ok else 'bad', 'text': text})
        finally:
            self.mode_busy = False

    def on_mode_request(self, cid, kind, msg):
        if self.start_mode not in ('patrol', 'explore'):
            return self.notice(cid, '모드 바꾸기는 app.launch.py 로 켰을 때만 됩니다')
        if self.mode_busy:
            return self.notice(cid, '모드를 바꾸는 중입니다. 잠시 기다려 주세요')
        if kind == 'explore_start':
            if self.modes.snapshot()['mode'] == 'explore':
                return self.notice(cid, '이미 지도 만들기 모드입니다')
            if self.runner.running():
                self.web.broadcast({'type': 'notice', 'level': 'warn', 'text': '순찰을 멈추고 지도 만들기를 시작합니다'})
            self.switch_mode_async('explore')
        elif kind == 'map_use':                         # 탐사를 멈추거나 끝난 뒤 순찰 모드로
            which = msg.get('which')
            path = self.explored_map + '.yaml' if which == 'explored' else self.default_map
            if which == 'explored' and not os.path.isfile(path):
                return self.notice(cid, '탐사로 만든 지도가 아직 없습니다')
            self.switch_mode_async('patrol', path)

    def mode_state(self):
        snap = self.modes.snapshot()
        exploring = snap['mode'] == 'explore' and (self.explore or {}).get('state') not in ('done',)
        return {'managed': self.start_mode in ('patrol', 'explore'), 'mode': snap['mode'], 'busy': self.mode_busy,
                'crashed': snap['exited'], 'exploring': exploring,
                'map': os.path.basename(self.map_file) if snap['mode'] == 'patrol' else None,
                'map_kind': ('explored' if self.map_file == self.explored_map + '.yaml'
                             else 'default' if self.map_file == self.default_map else 'custom'),
                'explored_ready': os.path.isfile(self.explored_map + '.yaml'),
                'explore': self.explore if snap['mode'] == 'explore' else None}

    def on_detection(self, msg):
        try:
            self.board.update(json.loads(msg.data), self.now())
        except json.JSONDecodeError:
            pass

    # ---------- 앱 ----------

    def on_app_connect(self, cid):
        self.web.send_to(cid, {'type': 'hello', 'id': cid})     # 앱이 자기가 조종 중인지 알 수 있게
        self.web.send_to(cid, {'type': 'courses', 'courses': self.courses})
        if self.map_msg is not None:
            self.web.send_to(cid, self.map_msg)

    def refresh_courses(self):
        self.course_paths, self.courses = list_courses(self.course_dirs)
        self.get_logger().info(f'순찰 코스 {len(self.courses)}개: {", ".join(c["id"] for c in self.courses) or "없음"} '
                               f'({", ".join(self.course_dirs)})')

    def notice(self, cid, text, level='warn'):
        self.web.send_to(cid, {'type': 'notice', 'level': level, 'text': text})

    def on_patrol_start(self, cid, msg):
        course = msg.get('course')
        try:
            laps = max(0, min(99, int(msg.get('laps', 0))))
        except (TypeError, ValueError):
            laps = 0
        if course not in self.course_paths:
            return self.notice(cid, '없는 코스입니다. 목록을 새로 고쳐 주세요')
        if self.mode_busy or self.modes.snapshot()['mode'] == 'explore':
            return self.notice(cid, '지도 만들기 모드에서는 순찰할 수 없습니다. 지도 탭에서 순찰 모드로 바꾸세요')
        if self.count_publishers('patrol/status') > 0 and not self.runner.running():
            return self.notice(cid, '터미널에서 켠 순찰이 이미 돌고 있습니다. 그 터미널에서 Ctrl+C 로 멈춘 뒤 시작하세요')
        with self.drive_lock:
            if self.drive.owner is not None:
                self.drive.stop('patrol')
        result = self.runner.start(course, self.course_paths[course], laps, self.sim)
        if result == 'running':
            self.notice(cid, '이미 순찰 중입니다')
        elif result == 'failed':
            self.notice(cid, '순찰을 켜지 못했습니다. 앱 서버 터미널의 오류를 확인하세요', 'bad')
        else:
            self.patrol = None                  # 지난 순찰의 진행 정보를 지운다
            self.patrol_running = True
            self.web.broadcast({'type': 'notice', 'level': 'ok', 'text': '순찰을 시작합니다'})

    def on_course_plan(self, cid, msg):
        """지도로 코스를 만들어 그 앱에만 미리 보여 준다. 계산(약 1초)은 따로 돌려 화면이 멈추지 않게."""
        try:
            view = max(1.0, min(5.0, float(msg.get('view', 2.5))))
        except (TypeError, ValueError):
            view = 2.5
        if self.map_raw is None:
            return self.notice(cid, '아직 지도를 받지 못했습니다')
        if self.robot_pose is None:
            return self.notice(cid, '로봇 위치를 모릅니다 (위치추정이 된 뒤 다시 시도하세요)')
        if self.planning:
            return self.notice(cid, '코스를 만드는 중입니다. 잠시 뒤 다시 누르세요')
        self.planning = True
        threading.Thread(target=self.make_plan, args=(cid, view, self.map_raw, self.robot_pose[:2]),
                         daemon=True).start()

    def make_plan(self, cid, view, map_raw, start):
        t0 = time.time()
        try:
            plan = plan_course(*map_raw, start, view_range=view)
        except ValueError as e:
            self.notice(cid, f'코스를 만들지 못했습니다: {e}', 'bad')
            return
        finally:
            self.planning = False
        plan.update(id='__plan__', name='자동 코스 (미리보기)', dwell=2.0, view=view)
        self.plans[cid] = plan
        self.get_logger().info(f'코스 자동 만들기: 감시 거리 {view} m → 지점 {len(plan["points"])}개, '
                               f'한 바퀴 {plan["length"]} m, 보이는 공간 {plan["coverage"] * 100:.0f}% '
                               f'({time.time() - t0:.1f}초)')
        self.web.send_to(cid, {'type': 'course_plan', 'plan': plan})

    def on_course_save(self, cid, msg):
        plan = self.plans.get(cid)
        if plan is None:
            return self.notice(cid, '저장할 미리보기 코스가 없습니다')
        now = datetime.datetime.now()
        name = str(msg.get('name') or '').strip()[:30] or f'자동 코스 {now:%m/%d %H:%M}'
        os.makedirs(self.save_dir, exist_ok=True)
        path = os.path.join(self.save_dir, f'auto_{now:%Y%m%d_%H%M%S}.yaml')
        text = route_yaml([(p['name'], p['x'], p['y']) for p in plan['points']], plan['dwell'], name=name,
                          header=f'관제 앱이 지도에서 자동으로 만든 코스 (감시 거리 {plan["view"]} m)')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        self.get_logger().info(f'자동 코스 저장: {path}')
        self.plans.pop(cid, None)
        self.refresh_courses()
        self.web.broadcast({'type': 'courses', 'courses': self.courses})
        self.web.send_to(cid, {'type': 'course_saved', 'id': os.path.splitext(os.path.basename(path))[0]})
        self.notice(cid, f"'{name}' 코스를 저장했습니다", 'ok')

    def on_patrol_stop(self, cid):
        if self.runner.running():
            self.runner.stop()
            self.web.broadcast({'type': 'notice', 'level': 'ok', 'text': '순찰을 멈추는 중입니다 (이동 취소)'})
        elif self.count_publishers('patrol/status') > 0:
            self.notice(cid, '터미널에서 켠 순찰입니다. 그 터미널에서 Ctrl+C 로 멈추세요')

    def on_app_message(self, cid, msg):
        """웹 서버 스레드에서 불린다. 여기서는 값만 바꾸고, 실제 발행은 drive_tick 이 한다."""
        kind = msg.get('type')
        log = self.get_logger()
        if kind == 'patrol_start':
            return self.on_patrol_start(cid, msg)
        if kind == 'patrol_stop':
            return self.on_patrol_stop(cid)
        if kind == 'course_plan':
            return self.on_course_plan(cid, msg)
        if kind == 'course_save':
            return self.on_course_save(cid, msg)
        if kind in ('explore_start', 'map_use'):
            return self.on_mode_request(cid, kind, msg)
        if kind == 'courses':
            self.refresh_courses()
            return self.web.send_to(cid, {'type': 'courses', 'courses': self.courses})
        with self.drive_lock:
            before = self.drive.owner
            if kind == 'disconnected':
                self.drive.disconnect(cid)
                self.plans.pop(cid, None)
            elif kind == 'stop':
                self.drive.stop()
            elif kind == 'drive':
                if self.patrol_running or self.mode_busy or self.exploring:
                    return                      # 순찰·탐사 중이거나 모드를 바꾸는 중에는 받지 않는다 (앱 화면에 이유 표시)
                self.drive.command(cid, msg.get('lin'), msg.get('ang'), time.monotonic())
            else:
                log.info(f'앱 #{cid} 명령 (아직 지원 안 함): {kind}')
            after = self.drive.owner
        if before != after:
            log.info(f'수동 주행 시작 (앱 #{after})' if after is not None
                     else f'수동 주행 정지 (앱 #{before}, {self.drive.reason})')

    # ---------- 수동 주행 발행 ----------

    def drive_tick(self):
        now = time.monotonic()
        with self.drive_lock:
            if (self.patrol_running or self.exploring) and self.drive.owner is not None:
                self.drive.stop('patrol')
                self.get_logger().warn('순찰·탐사가 켜져서 수동 주행을 멈춤')
            before = self.drive.owner
            out = self.drive.step(now)
            timed_out = before is not None and self.drive.owner is None
        if timed_out:
            self.get_logger().warn(f'앱 #{before} 명령이 {self.drive.timeout:.1f}초 끊겨 정지 (Wi-Fi 끊김 등)')
        if out is None:
            self.blocked = None
            return
        linear, angular = out
        points = self.scan_points(step=1) if now - self.scan_at < 1.0 else []   # 오래된 라이다로는 막지 않는다
        linear, self.blocked = guard_speed(linear, points, self.guard_gap)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """노드를 끌 때: 수동 주행 중이었으면 0 속도를 보낸다."""
        with self.drive_lock:
            moving = self.drive.owner is not None or self.drive.zero_left > 0
            self.drive.stop('shutdown')
        if moving:
            try:
                self.cmd_pub.publish(Twist())
            except Exception:  # noqa: BLE001 - 이미 ROS 통신이 닫힌 경우
                pass

    # ---------- 주기 전송 ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_points(self, step):
        """최근 라이다 → 로봇 기준 점. 라이다 위치(TF)를 아직 모르면 빈 목록."""
        s = self.scan
        if s is None:
            return []
        if self.laser_pose is None:
            try:
                self.laser_pose = self.lookup(self.base_frame, s.header.frame_id)
            except TransformException:
                return []
        return scan_to_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                              *self.laser_pose, step=step)

    def tick(self):
        # 순찰 노드가 살아 있는지: patrol/status 를 내는 쪽(patrol_node)이 있으면 순찰 중
        external = self.count_publishers('patrol/status') > 0 and not self.runner.running()
        self.patrol_running = self.runner.running() or self.count_publishers('patrol/status') > 0
        mode = self.mode_state()
        self.exploring = mode['exploring']
        self.send_initial_pose()
        # 로봇 위치는 앱이 없어도 갱신한다 (모드를 바꿀 때 탐사가 끝난 위치를 기억해야 한다)
        try:
            robot, localized = self.lookup(self.map_frame, self.base_frame), True
            self.robot_pose = robot
        except TransformException:
            robot, localized = None, False
        if self.web.client_count() == 0:
            return
        points = self.scan_points(self.scan_step)
        now = self.now()
        with self.drive_lock:
            manual = self.drive.snapshot()
        manual['blocked'] = self.blocked
        patrol_app = self.runner.snapshot()
        patrol_app['owner'] = 'app' if patrol_app['running'] else 'external' if external else None
        self.web.broadcast(build_state(robot, points, self.battery, self.rssi, self.patrol,
                                       self.board.snapshot(now), localized, now,
                                       manual=manual, patrol_running=self.patrol_running,
                                       patrol_app=patrol_app, mode=mode))


def main():
    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 닫아 마지막 정지 명령을 못 보낸다 → 직접 처리 (patrol_node 와 같은 방식)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AppServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.runner.stop(wait=True)          # 앱이 켠 순찰도 함께 끈다 (이동 취소)
        node.modes.stop()                    # 앱이 켠 모드(Nav2·감지·탐사)도 함께 끈다
        node.stop_robot()
        node.web.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
