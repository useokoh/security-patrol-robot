"""1단계 점검용 노드: /scan 과 /odom 이 제대로 들어오는지 주기적으로 요약해서 출력한다."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanCheck(Node):

    def __init__(self):
        super().__init__('scan_check')
        self.declare_parameter('report_period', 2.0)
        self.period = float(self.get_parameter('report_period').value)

        self.scan_count = 0
        self.odom_count = 0
        self.last_scan = None
        self.last_odom = None

        # 센서 데이터는 best-effort 로 구독해야 reliable/best-effort 발행자 모두와 연결된다.
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(self.period, self.report)
        self.get_logger().info(f'/scan, /odom 점검 시작 ({self.period:.1f}초마다 보고)')

    def on_scan(self, msg):
        self.scan_count += 1
        self.last_scan = msg

    def on_odom(self, msg):
        self.odom_count += 1
        self.last_odom = msg

    def report(self):
        log = self.get_logger()

        if self.scan_count == 0:
            log.warn(f'지난 {self.period:.1f}초 동안 /scan 수신 없음')
        else:
            s = self.last_scan
            valid = [
                (i, r) for i, r in enumerate(s.ranges)
                if math.isfinite(r) and s.range_min <= r <= s.range_max
            ]
            if valid:
                i, r = min(valid, key=lambda v: v[1])
                angle = math.degrees(s.angle_min + i * s.angle_increment)
                nearest = f'가장 가까운 물체 {r:.2f} m @ {angle:+.0f}°'
            else:
                nearest = '유효한 거리값 없음 (주변이 범위 밖?)'
            log.info(
                f'[scan] {self.scan_count / self.period:.1f} Hz | frame={s.header.frame_id} | '
                f'유효 {len(valid)}/{len(s.ranges)}점 | {nearest}'
            )

        if self.odom_count == 0:
            log.warn(f'지난 {self.period:.1f}초 동안 /odom 수신 없음')
        else:
            p = self.last_odom.pose.pose
            yaw = math.degrees(yaw_from_quaternion(p.orientation))
            log.info(
                f'[odom] {self.odom_count / self.period:.1f} Hz | '
                f'x={p.position.x:+.2f} y={p.position.y:+.2f} yaw={yaw:+.0f}°'
            )

        self.scan_count = 0
        self.odom_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
