"""지도 작성용 자동 주행 노드: 웨이포인트 목록을 odom 기준으로 차례로 찾아간다.

- 목표 방향과 각도 차이가 크면 제자리 회전, 작으면 전진하면서 방향을 보정한다 (단순 P 제어).
- 정면(±30°)에 장애물이 stop_distance 보다 가까우면 전진을 멈추고, 오래 막히면 그 웨이포인트를 건너뛴다.
- Nav2 없이 동작하므로 지도가 아직 없는 2단계에서 쓸 수 있다. (3단계 이후에는 Nav2 로 대체)

  ros2 run security_patrol auto_mapper --ros-args -p use_sim_time:=true \
      --params-file ~/security_patrol_ws/install/security_patrol/share/security_patrol/config/mapping_route.yaml
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutoMapper(Node):

    def __init__(self):
        super().__init__('auto_mapper')
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('turn_in_place_angle', 0.35)  # rad, 이보다 크면 제자리 회전
        self.declare_parameter('stop_distance', 0.25)        # m, 정면 장애물 정지 거리
        self.declare_parameter('blocked_timeout', 10.0)      # s, 이만큼 막히면 웨이포인트 건너뜀

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 2 or len(flat) % 2:
            raise ValueError('waypoints 는 [x1, y1, x2, y2, ...] 형식이어야 합니다 (--params-file 확인)')
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]

        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)
        self.tol = float(self.get_parameter('goal_tolerance').value)
        self.turn_angle = float(self.get_parameter('turn_in_place_angle').value)
        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.blocked_timeout = float(self.get_parameter('blocked_timeout').value)

        self.pose = None          # (x, y, yaw)
        self.front_min = math.inf
        self.index = 0
        self.blocked_since = None
        self.start_time = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info(f'웨이포인트 {len(self.waypoints)}개 주행 준비, odom 수신 대기 중...')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_scan(self, msg):
        # 라이다 프레임 기준 정면 ±30° 안의 가장 가까운 거리
        front = math.radians(30)
        nearest = math.inf
        for i, r in enumerate(msg.ranges):
            angle = wrap_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) <= front and math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                nearest = min(nearest, r)
        self.front_min = nearest

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def next_waypoint(self, reason):
        x, y = self.waypoints[self.index]
        self.get_logger().info(f'[{self.index + 1}/{len(self.waypoints)}] ({x:+.1f}, {y:+.1f}) {reason}')
        self.index += 1
        self.blocked_since = None

    def step(self):
        if self.pose is None:
            return
        if self.start_time is None:
            self.start_time = self.now_sec()
            self.get_logger().info('주행 시작')

        if self.index >= len(self.waypoints):
            self.finish()
            return

        x, y, yaw = self.pose
        gx, gy = self.waypoints[self.index]
        dist = math.hypot(gx - x, gy - y)
        if dist < self.tol:
            self.next_waypoint('도착')
            return

        heading_err = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        cmd = Twist()
        if abs(heading_err) > self.turn_angle:
            # 방향이 많이 틀어졌으면 제자리 회전 (회전은 장애물과 무관하게 안전)
            cmd.angular.z = math.copysign(self.ang, heading_err)
            self.blocked_since = None
        elif self.front_min < self.stop_dist:
            # 정면이 막혔으면 정지하고 기다리다가, 오래 막히면 건너뜀
            if self.blocked_since is None:
                self.blocked_since = self.now_sec()
                self.get_logger().warn(f'정면 장애물 {self.front_min:.2f} m → 대기')
            elif self.now_sec() - self.blocked_since > self.blocked_timeout:
                self.next_waypoint('막혀서 건너뜀')
                return
        else:
            # 전진하면서 방향 보정 (가까워지면 감속)
            cmd.linear.x = min(self.lin, 0.5 * dist + 0.05)
            cmd.angular.z = max(-self.ang, min(self.ang, 2.0 * heading_err))
            self.blocked_since = None
        self.cmd_pub.publish(cmd)

    def finish(self):
        self.timer.cancel()
        self.cmd_pub.publish(Twist())
        elapsed = self.now_sec() - self.start_time
        self.get_logger().info(f'=== 경로 주행 완료 ({elapsed:.0f}초) → 이제 지도를 저장하세요 ===')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
