"""8단계 실제 로봇 연결 노드: ESP32 원시 데이터 → /scan, /odom, odom→base_link TF, /joint_states.

시뮬레이션에서는 Gazebo 가 /scan 과 /odom 을 바로 내주지만, 실제 로봇(ESP32)은 대역폭 때문에
라이다를 4조각의 바이트 배열로, 바퀴는 엔코더 변화량으로 보낸다. 이 노드가 그것을 시뮬레이션과
같은 토픽·프레임으로 바꿔 주므로 SLAM, Nav2, 이상 감지 노드는 그대로 쓸 수 있다.
계산은 robot_bridge_logic.py 에 있다.

  ros2 launch security_patrol real_bringup.launch.py   # micro-ROS agent + 이 노드 + 로봇 모델
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32, Int32, Int64MultiArray, UInt8MultiArray
from tf2_ros import TransformBroadcaster

from security_patrol.robot_bridge_logic import (SCAN_SIZE, OdomIntegrator, ScanAssembler,
                                                telemetry_extras, yaw_to_quaternion)

WHEEL_JOINTS = ['front_left_wheel_joint', 'rear_left_wheel_joint',
                'front_right_wheel_joint', 'rear_right_wheel_joint']


class RobotBridge(Node):

    def __init__(self):
        super().__init__('robot_bridge')
        p = self.declare_parameter
        self.odom = OdomIntegrator(
            ticks_per_rev=float(p('ticks_per_rev', 7.0 * 298 * 4).value),
            wheel_radius=float(p('wheel_radius', 0.065 / 2).value),
            gyro_lsb_per_dps=float(p('gyro_lsb_per_dps', 131.0).value),
            idle_ticks=float(p('idle_ticks', 3.0).value))
        self.laser_frame = p('laser_frame', 'laser_frame').value
        self.scan_hz = float(p('scan_hz', 6.0).value)
        # 4조각이 다 모인 시각은 한 바퀴(약 0.17초)가 끝난 뒤다. SLAM 과 AMCL 은 스캔 전체를 한 시각으로 보므로
        # 한 바퀴의 가운데 시각으로 당겨 붙인다 (회전 중 0.6 rad/s 면 0.17초에 약 6° 차이가 난다)
        self.scan_delay = Duration(seconds=float(p('scan_delay', 0.5 / self.scan_hz).value))
        self.publish_tf = bool(p('publish_tf', True).value)
        self.assembler = ScanAssembler()

        # 펌웨어가 best effort 로 보내므로 받는 쪽도 맞춘다
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UInt8MultiArray, 'range_segment', self.on_segment, best_effort)
        self.create_subscription(Int64MultiArray, 'encoder_deltas_gyroz_bat_rssi', self.on_telemetry,
                                 best_effort)

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.rssi_pub = self.create_publisher(Int32, 'rssi_strength', 10)
        self.tf = TransformBroadcaster(self)

        self.counts = {'scan': 0, 'odom': 0}
        self.battery = None
        self.rssi = None
        self.create_timer(5.0, self.report)
        self.get_logger().info('ESP32 데이터를 기다리는 중 (range_segment, encoder_deltas_gyroz_bat_rssi)')

    # ---------- 라이다 ----------

    def on_segment(self, msg):
        ranges = self.assembler.add(msg.data)
        if ranges is None:
            return
        scan = LaserScan()
        scan.header.stamp = (self.get_clock().now() - self.scan_delay).to_msg()
        scan.header.frame_id = self.laser_frame
        # 펌웨어가 반시계 방향으로 정렬해서 보내고, 첫 점이 로봇 뒤쪽(-180°)이다 (강의 코드 기준)
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / SCAN_SIZE
        scan.angle_max = scan.angle_min + scan.angle_increment * (SCAN_SIZE - 1)
        scan.scan_time = 1.0 / self.scan_hz
        scan.time_increment = scan.scan_time / SCAN_SIZE
        scan.range_min = 0.15
        scan.range_max = 8.0
        scan.ranges = ranges        # 0.0(측정 실패)은 range_min 보다 작아 다른 노드가 무시한다
        self.scan_pub.publish(scan)
        self.counts['scan'] += 1

    # ---------- 바퀴·자이로 ----------

    def on_telemetry(self, msg):
        values = list(msg.data)
        if len(values) >= 9:
            self.battery, self.rssi = telemetry_extras(values)
            self.battery_pub.publish(Float32(data=float(self.battery)))
            self.rssi_pub.publish(Int32(data=self.rssi))
        if not self.odom.update(values):
            return
        stamp = self.get_clock().now().to_msg()
        o = self.odom
        qx, qy, qz, qw = yaw_to_quaternion(o.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = o.x
        odom.pose.pose.position.y = o.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = o.linear
        odom.twist.twist.angular.z = o.angular
        self.odom_pub.publish(odom)
        self.counts['odom'] += 1

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = WHEEL_JOINTS
        joints.position = list(o.wheel_angles)
        self.joint_pub.publish(joints)

    def report(self):
        """5초마다 수신 상태를 알린다. 데이터가 안 오면 연결 문제를 바로 알 수 있다."""
        scan_hz = self.counts['scan'] / 5.0
        odom_hz = self.counts['odom'] / 5.0
        self.counts = {'scan': 0, 'odom': 0}
        if scan_hz == 0 and odom_hz == 0:
            self.get_logger().warn('ESP32 데이터가 오지 않음: 로봇 전원, 핫스팟, 중계 프로그램, agent 를 확인')
            return
        extra = ''
        if self.battery is not None:
            extra = f', 배터리 {self.battery:.2f} V, Wi-Fi {self.rssi} dBm'
        self.get_logger().info(
            f'scan {scan_hz:.1f} Hz, odom {odom_hz:.0f} Hz{extra}, '
            f'위치 ({self.odom.x:+.2f}, {self.odom.y:+.2f}, {math.degrees(self.odom.yaw):+.0f}°), '
            f'버린 스캔 누적 {self.assembler.dropped}')


def main():
    rclpy.init()
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
