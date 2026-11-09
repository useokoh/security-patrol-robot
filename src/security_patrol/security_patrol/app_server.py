"""앱 서버 노드: 로봇 상태를 모아 웹앱(휴대폰·맥 브라우저)에 보내고, 앱의 명령을 받는다.

1단계: 레이더 화면 - 지도, 로봇 위치, 라이다 점, 감지 물체, 순찰 상태, 배터리·Wi-Fi
2단계: 수동 주행 - 조이스틱 → cmd_vel. 손을 떼거나, 명령이 0.5초 끊기거나, 연결이 끊기면 정지.
       앞뒤가 막히면 전후진만 막고, 순찰이 켜져 있으면 수동 주행을 받지 않는다.
3단계부터: 코스 선택·자동 순찰

  # 시뮬레이션
  ros2 run security_patrol app_server --ros-args -p use_sim_time:=true
  # 실제 로봇 (4륜이 미끄러져 회전이 명령의 약 25% 라 회전 상한을 올린다)
  ros2 run security_patrol app_server --ros-args -p max_angular:=1.8
  # 브라우저: 가상머신 안 http://localhost:8080, 맥 http://192.168.64.2:8080
"""

import json
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
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
                                       scan_to_points, yaw_from_quaternion)
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
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.web = WebServer(web_dir, port=port, on_message=self.on_app_message,
                             on_connect=self.on_app_connect, log=self.get_logger().info).start_in_thread()
        self.create_timer(1.0 / rate, self.tick)
        # 수동 주행 정지 판단은 실제 시간으로 한다 (시뮬레이션 시계가 멈추거나 느려도 끊김을 잡도록)
        self.create_timer(1.0 / drive_rate, self.drive_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'앱 서버 시작: http://<이 컴퓨터 주소>:{self.web.port} (화면 파일 {web_dir})')

    # ---------- ROS 입력 ----------

    def on_map(self, msg):
        info = msg.info
        q = info.origin.orientation
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

    def on_detection(self, msg):
        try:
            self.board.update(json.loads(msg.data), self.now())
        except json.JSONDecodeError:
            pass

    # ---------- 앱 ----------

    def on_app_connect(self, cid):
        self.web.send_to(cid, {'type': 'hello', 'id': cid})     # 앱이 자기가 조종 중인지 알 수 있게
        if self.map_msg is not None:
            self.web.send_to(cid, self.map_msg)

    def on_app_message(self, cid, msg):
        """웹 서버 스레드에서 불린다. 여기서는 값만 바꾸고, 실제 발행은 drive_tick 이 한다."""
        kind = msg.get('type')
        log = self.get_logger()
        with self.drive_lock:
            before = self.drive.owner
            if kind == 'disconnected':
                self.drive.disconnect(cid)
            elif kind == 'stop':
                self.drive.stop()
            elif kind == 'drive':
                if self.patrol_running:
                    return                      # 순찰 중에는 받지 않는다 (앱 화면에 이유 표시)
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
            if self.patrol_running and self.drive.owner is not None:
                self.drive.stop('patrol')
                self.get_logger().warn('순찰이 켜져서 수동 주행을 멈춤')
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
        self.patrol_running = self.count_publishers('patrol/status') > 0
        if self.web.client_count() == 0:
            return
        points = self.scan_points(self.scan_step)
        try:
            robot, localized = self.lookup(self.map_frame, self.base_frame), True
        except TransformException:
            robot, localized = None, False
        now = self.now()
        with self.drive_lock:
            manual = self.drive.snapshot()
        manual['blocked'] = self.blocked
        self.web.broadcast(build_state(robot, points, self.battery, self.rssi, self.patrol,
                                       self.board.snapshot(now), localized, now,
                                       manual=manual, patrol_running=self.patrol_running))


def main():
    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 닫아 마지막 정지 명령을 못 보낸다 → 직접 처리 (patrol_node 와 같은 방식)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AppServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.web.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
