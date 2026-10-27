"""위치추정 점검 노드: AMCL 추정 위치(/amcl_pose)를 실제 위치와 비교해 주기적으로 출력한다.

- 시뮬레이션에서는 /odom 이 Gazebo 의 실제 위치(월드 좌표 = map 좌표)라서 정답으로 쓸 수 있다.
  실제 로봇에서는 /odom 에 바퀴 오차가 쌓이므로 '오차' 대신 σ(AMCL 불확실성)만 참고한다.
- 위치를 잃는 순간(오차가 기준을 넘는 순간)에 경고를 한 번 출력하고, 종료 시 요약을 출력한다.
- 함께 출력하는 scan Hz, RTF(시뮬레이션 속도), AMCL 갱신 횟수로 CPU 부족 여부를 확인할 수 있다.

  ros2 run security_patrol loc_monitor --ros-args -p use_sim_time:=true
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """a - b 를 -π ~ π 범위로."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class LocMonitor(Node):

    def __init__(self):
        super().__init__('loc_monitor')
        self.declare_parameter('report_period', 1.0)       # 보고 주기 [초, 시뮬레이션 시간]
        self.declare_parameter('lost_xy', 0.5)             # 이 거리 이상 틀리면 '위치 잃음' [m]
        self.declare_parameter('lost_yaw_deg', 20.0)       # 이 각도 이상 틀리면 '위치 잃음' [deg]
        self.period = float(self.get_parameter('report_period').value)
        self.lost_xy = float(self.get_parameter('lost_xy').value)
        self.lost_yaw = math.radians(float(self.get_parameter('lost_yaw_deg').value))

        self.amcl = None          # 가장 최근 AMCL 추정
        self.truth = None         # 가장 최근 /odom
        self.amcl_count = 0       # 보고 주기 동안 AMCL 갱신 횟수
        self.scan_count = 0
        self.lost = False
        self.lost_events = 0
        self.max_err_xy = 0.0
        self.max_err_yaw = 0.0
        self.start_sim = None
        self.last_sim = None
        self.last_wall = None

        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(self.period, self.report)
        self.get_logger().info(
            f'위치추정 점검 시작 (기준: {self.lost_xy:.2f} m / '
            f'{math.degrees(self.lost_yaw):.0f}° 이상 틀리면 위치 잃음)')

    def on_amcl(self, msg):
        self.amcl = msg
        self.amcl_count += 1

    def on_odom(self, msg):
        self.truth = msg

    def on_scan(self, _msg):
        self.scan_count += 1

    def report(self):
        log = self.get_logger()
        now_sim = self.get_clock().now().nanoseconds / 1e9
        now_wall = time.monotonic()
        if self.start_sim is None:
            self.start_sim = now_sim

        # 시뮬레이션 속도(RTF) = 시뮬레이션 시간 경과 / 실제 시간 경과. 1.0 보다 많이 작으면 CPU 부족
        rtf = ''
        if self.last_wall is not None and now_wall > self.last_wall:
            rtf = f' | RTF {(now_sim - self.last_sim) / (now_wall - self.last_wall):.2f}'
        self.last_sim, self.last_wall = now_sim, now_wall
        stamp = f'[{now_sim - self.start_sim:5.0f}s]'
        stats = f'AMCL {self.amcl_count}회 | scan {self.scan_count / self.period:.1f}Hz{rtf}'
        self.amcl_count = 0
        self.scan_count = 0

        if self.amcl is None or self.truth is None:
            log.info(f'{stamp} 대기 중 (amcl_pose={self.amcl is not None}, '
                     f'odom={self.truth is not None}) | {stats}')
            return

        a = self.amcl.pose.pose
        t = self.truth.pose.pose
        err_xy = math.hypot(a.position.x - t.position.x, a.position.y - t.position.y)
        err_yaw = abs(angle_diff(yaw_from_quaternion(a.orientation),
                                 yaw_from_quaternion(t.orientation)))
        self.max_err_xy = max(self.max_err_xy, err_xy)
        self.max_err_yaw = max(self.max_err_yaw, err_yaw)

        # AMCL 스스로 생각하는 불확실성 (공분산 → 표준편차)
        cov = self.amcl.pose.covariance
        sigma_xy = math.sqrt(max(cov[0] + cov[7], 0.0))
        sigma_yaw = math.degrees(math.sqrt(max(cov[35], 0.0)))

        moving = '회전 중' if abs(self.truth.twist.twist.angular.z) > 0.2 else (
            '이동 중' if abs(self.truth.twist.twist.linear.x) > 0.02 else '정지')
        line = (f'{stamp} 오차 {err_xy:.2f}m {math.degrees(err_yaw):3.0f}° | '
                f'σ {sigma_xy:.2f}m {sigma_yaw:3.0f}° | {moving} | {stats}')

        is_lost = err_xy > self.lost_xy or err_yaw > self.lost_yaw
        if is_lost and not self.lost:
            self.lost_events += 1
            log.error(f'!!! 위치 잃음: 실제 ({t.position.x:+.2f}, {t.position.y:+.2f}) / '
                      f'AMCL ({a.position.x:+.2f}, {a.position.y:+.2f})')
        elif not is_lost and self.lost:
            log.info('위치 다시 찾음')
        self.lost = is_lost
        # rclpy 는 같은 줄의 로그 호출에서 심각도(warn/info)가 바뀌면 에러를 내므로 호출을 나눈다
        if is_lost:
            log.warn(line)
        else:
            log.info(line)

    def summary(self):
        self.get_logger().info(
            f'=== 요약: 최대 오차 {self.max_err_xy:.2f} m / {math.degrees(self.max_err_yaw):.0f}°, '
            f'위치 잃음 {self.lost_events}회 ===')


def main(args=None):
    rclpy.init(args=args)
    node = LocMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
