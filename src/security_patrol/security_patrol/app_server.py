"""앱 서버 노드: 로봇 상태를 모아 웹앱(휴대폰·맥 브라우저)에 보내고, 앱의 명령을 받는다.

1단계(지금): 레이더 화면 - 지도, 로봇 위치, 라이다 점, 감지 물체, 순찰 상태, 배터리·Wi-Fi
2단계부터: 수동 주행, 3단계부터: 코스 선택·자동 순찰

  # 시뮬레이션
  ros2 run security_patrol app_server --ros-args -p use_sim_time:=true
  # 실제 로봇
  ros2 run security_patrol app_server
  # 브라우저: 가상머신 안 http://localhost:8080, 맥 http://192.168.64.2:8080
"""

import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from security_patrol.app_logic import ObjectBoard, build_state, encode_map, scan_to_points, yaw_from_quaternion
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.scan = None
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

        self.web = WebServer(web_dir, port=port, on_message=self.on_app_message,
                             on_connect=self.on_app_connect, log=self.get_logger().info).start_in_thread()
        self.create_timer(1.0 / rate, self.tick)
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
        if self.map_msg is not None:
            self.web.send_to(cid, self.map_msg)

    def on_app_message(self, cid, msg):
        kind = msg.get('type')
        if kind == 'disconnected':
            return
        self.get_logger().info(f'앱 #{cid} 명령 (아직 지원 안 함): {kind}')

    # ---------- 주기 전송 ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def tick(self):
        if self.web.client_count() == 0:
            return
        points = []
        if self.scan is not None:
            if self.laser_pose is None:
                try:
                    self.laser_pose = self.lookup(self.base_frame, self.scan.header.frame_id)
                except TransformException:
                    pass
            if self.laser_pose is not None:
                s = self.scan
                points = scan_to_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                                        *self.laser_pose, step=self.scan_step)
        try:
            robot, localized = self.lookup(self.map_frame, self.base_frame), True
        except TransformException:
            robot, localized = None, False
        now = self.now()
        self.web.broadcast(build_state(robot, points, self.battery, self.rssi, self.patrol,
                                       self.board.snapshot(now), localized, now))


def main():
    rclpy.init()
    node = AppServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.web.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
