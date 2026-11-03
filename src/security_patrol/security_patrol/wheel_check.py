"""8단계 점검 노드: 실제 로봇의 바퀴 4개가 명령대로 도는지 바퀴별로 잰다.

전진 / 왼쪽 회전 / 오른쪽 회전을 차례로 시키고, ESP32 가 보낸 엔코더 변화량(tick)을 바퀴마다 더해
기대값과 비교한다. 배터리 전압과 자이로 회전각도 함께 보여 준다.
바퀴를 바닥에서 띄운 상태(상자 위)로 한 번, 바닥에서 한 번 돌리면 모터 문제와 바닥 마찰 문제를 가를 수 있다.

  ros2 run security_patrol wheel_check
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64MultiArray

NAMES = ['앞왼쪽', '뒤왼쪽', '앞오른쪽', '뒤오른쪽']


class WheelCheck(Node):

    def __init__(self):
        super().__init__('wheel_check')
        p = self.declare_parameter
        self.ticks_per_rev = float(p('ticks_per_rev', 7.0 * 298 * 4).value)
        self.wheel_radius = float(p('wheel_radius', 0.065 / 2).value)
        self.track = float(p('track', 0.145).value)            # 좌우 바퀴 사이 거리 [m] (URDF ±0.0725)
        self.speed = float(p('linear_speed', 0.1).value)
        self.turn = float(p('angular_speed', 1.0).value)
        self.duration = float(p('phase_time', 2.0).value)
        self.phases = [('전진', self.speed, 0.0), ('왼쪽 회전', 0.0, self.turn), ('오른쪽 회전', 0.0, -self.turn)]

        best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Int64MultiArray, 'encoder_deltas_gyroz_bat_rssi', self.on_telemetry, best_effort)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.index = -1           # -1 = 데이터 기다리는 중
        self.phase_start = None
        self.reset()
        self.battery = None
        self.messages = 0
        self.create_timer(0.05, self.tick)
        self.get_logger().info('로봇 데이터 기다리는 중... (바퀴가 움직입니다. 주변을 비워 두세요)')

    def reset(self):
        self.ticks = [0, 0, 0, 0]
        self.gyro_deg = 0.0
        self.count = 0
        self.prev_stamp = None

    def on_telemetry(self, msg):
        v = list(msg.data)
        if len(v) != 9:
            return
        self.messages += 1
        self.battery = v[7] / 1000.0
        if self.index < 0 or self.index >= len(self.phases):
            return
        stamp = v[0] + v[1] / 1e9
        if self.prev_stamp is not None and 0 < stamp - self.prev_stamp < 0.2:
            self.gyro_deg += v[6] / 131.0 * (stamp - self.prev_stamp)
        self.prev_stamp = stamp
        for i in range(4):
            self.ticks[i] += v[2 + i]
        self.count += 1

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

    def tick(self):
        if self.index < 0:
            if self.messages > 20:
                self.get_logger().info(f'배터리 {self.battery:.2f} V. 점검 시작')
                self.start_phase(0)
            return
        if self.index >= len(self.phases):
            return
        name, lin, ang = self.phases[self.index]
        elapsed = self.now() - self.phase_start
        if elapsed < self.duration:
            self.send(lin, ang)
            return
        self.send(0.0, 0.0)
        if elapsed < self.duration + 1.0:      # 멈춘 뒤 1초 동안 남은 움직임까지 센다
            return
        self.report(name, lin, ang)
        self.start_phase(self.index + 1)

    def start_phase(self, index):
        self.index = index
        self.reset()
        self.phase_start = self.now()
        if index < len(self.phases):
            self.get_logger().info(f'▶ {self.phases[index][0]} ({self.duration:.0f}초)')
        else:
            self.send(0.0, 0.0)
            self.get_logger().info('=== 끝. 결과를 Claude 에게 보여 주세요 ===')

    def report(self, name, lin, ang):
        circumference = 2 * math.pi * self.wheel_radius
        meters = [t / self.ticks_per_rev * circumference for t in self.ticks]
        # 기대 바퀴 이동: 왼쪽 = v - w*track/2, 오른쪽 = v + w*track/2
        left = (lin - ang * self.track / 2) * self.duration
        right = (lin + ang * self.track / 2) * self.duration
        expected = [left, left, right, right]
        parts = [f'{n} {m * 100:+.1f}cm(기대 {e * 100:+.1f})' for n, m, e in zip(NAMES, meters, expected)]
        self.get_logger().info(f'[{name}] ' + ', '.join(parts))
        self.get_logger().info(
            f'[{name}] tick {self.ticks}, 자이로 회전 {self.gyro_deg:+.0f}° '
            f'(기대 {math.degrees(ang * self.duration):+.0f}°), 메시지 {self.count}개, 배터리 {self.battery:.2f} V')


def main():
    rclpy.init()
    node = WheelCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
