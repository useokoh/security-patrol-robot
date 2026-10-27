"""자동 주행 테스트 노드: /cmd_vel 로 정해진 동작을 시키고 /odom 으로 실제 움직임을 재서 PASS/FAIL 을 출력한다.

동작 순서 (각 단계 사이 1초 정지):
  1. 전진        linear_speed  로 phase_time 초  → 이동 거리 확인
  2. 왼쪽 회전   +angular_speed 로 phase_time 초 → 회전 각도 확인 (제자리 회전)
  3. 오른쪽 회전 -angular_speed 로 phase_time 초 → 회전 각도 확인

실제 움직임이 명령값의 pass_ratio 이상이면 PASS.
로봇 앞쪽에 phase_time * linear_speed 이상 빈 공간이 있어야 한다.

  ros2 run security_patrol motion_test --ros-args -p use_sim_time:=true
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MotionTest(Node):

    def __init__(self):
        super().__init__('motion_test')
        self.declare_parameter('linear_speed', 0.1)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        self.declare_parameter('phase_time', 3.0)      # 초
        self.declare_parameter('pass_ratio', 0.7)      # 명령 대비 실제 움직임 최소 비율

        lin = float(self.get_parameter('linear_speed').value)
        ang = float(self.get_parameter('angular_speed').value)
        t = float(self.get_parameter('phase_time').value)
        self.pass_ratio = float(self.get_parameter('pass_ratio').value)

        # (이름, 선속도, 각속도, 지속시간, 측정 종류)
        self.phases = [
            ('전진', lin, 0.0, t, 'distance'),
            ('정지', 0.0, 0.0, 1.0, None),
            ('왼쪽 제자리 회전', 0.0, ang, t, 'yaw'),
            ('정지', 0.0, 0.0, 1.0, None),
            ('오른쪽 제자리 회전', 0.0, -ang, t, 'yaw'),
            ('정지', 0.0, 0.0, 1.0, None),
        ]
        self.index = -1
        self.phase_start_time = None
        self.results = []

        # odom 누적값 (yaw 는 ±180° 경계를 넘어도 이어지도록 누적)
        self.odom_received = False
        self.x = self.y = 0.0
        self.total_yaw = 0.0
        self.last_yaw = None
        self.start_x = self.start_y = self.start_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info('odom 수신 대기 중...')

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        if self.last_yaw is not None:
            self.total_yaw += wrap_angle(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.x, self.y = p.position.x, p.position.y
        self.odom_received = True

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def start_phase(self, index):
        self.index = index
        self.phase_start_time = self.now_sec()
        self.start_x, self.start_y, self.start_yaw = self.x, self.y, self.total_yaw
        name = self.phases[index][0]
        if name != '정지':
            self.get_logger().info(f'▶ {name} 시작')

    def finish_phase(self):
        name, lin, ang, duration, kind = self.phases[self.index]
        if kind == 'distance':
            actual = math.hypot(self.x - self.start_x, self.y - self.start_y)
            expected = abs(lin) * duration
            unit, shown_actual, shown_expected = 'm', actual, expected
        elif kind == 'yaw':
            actual = abs(self.total_yaw - self.start_yaw)
            expected = abs(ang) * duration
            unit, shown_actual, shown_expected = '°', math.degrees(actual), math.degrees(expected)
        else:
            return
        ratio = actual / expected if expected > 0 else 0.0
        ok = ratio >= self.pass_ratio
        self.results.append((name, ok))
        self.get_logger().info(
            f'  {"PASS" if ok else "FAIL"} {name}: 명령 {shown_expected:.2f}{unit} → '
            f'실제 {shown_actual:.2f}{unit} ({ratio * 100:.0f}%)')

    def step(self):
        if not self.odom_received:
            return
        if self.index < 0:
            self.start_phase(0)

        _, lin, ang, duration, _ = self.phases[self.index]
        if self.now_sec() - self.phase_start_time >= duration:
            self.finish_phase()
            if self.index + 1 >= len(self.phases):
                self.report()
                return
            self.start_phase(self.index + 1)
            _, lin, ang, _, _ = self.phases[self.index]

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    def report(self):
        self.timer.cancel()
        self.cmd_pub.publish(Twist())  # 확실히 정지
        passed = sum(ok for _, ok in self.results)
        verdict = 'ALL PASS' if passed == len(self.results) else 'FAIL 있음'
        self.get_logger().info(f'=== 결과: {passed}/{len(self.results)} 통과 → {verdict} ===')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = MotionTest()
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
