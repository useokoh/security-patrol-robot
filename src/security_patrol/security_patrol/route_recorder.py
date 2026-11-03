"""8단계: 실제 공간의 순찰 지점을 기록한다.

지도를 만들며 키보드로 로봇을 몰다가, 순찰 지점으로 삼을 곳에서 Enter 를 누르면
그 순간의 로봇 위치(map 좌표)를 patrol_route.yaml 형식으로 저장한다.
로봇이 실제로 서 있던 곳이라 Nav2 가 갈 수 있는 지점만 남는다.

사용 예 (real_slam.launch.py 가 켜져 있는 상태, 키보드 조종과 다른 터미널에서):
  ros2 run security_patrol route_recorder
  ros2 run security_patrol route_recorder --ros-args -p output:=$HOME/maps_real/patrol_route_real.yaml
입력: Enter = '지점N' 으로 기록, 이름 입력 후 Enter = 그 이름으로 기록, u = 마지막 지점 지우기, q = 저장하고 끝내기
"""

import math
import os
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from security_patrol.route_file import route_yaml

DEFAULT_OUTPUT = os.path.expanduser('~/maps_real/patrol_route_real.yaml')


class RouteRecorder(Node):

    def __init__(self):
        super().__init__('route_recorder')
        self.output = os.path.expanduser(self.declare_parameter('output', DEFAULT_OUTPUT).value)
        self.min_gap = float(self.declare_parameter('min_gap', 0.3).value)   # 너무 가까운 지점 경고 [m]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = []

    def robot_xy(self):
        tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def record(self, name):
        try:
            x, y = self.robot_xy()
        except TransformException as e:
            print(f'  위치를 아직 모릅니다 (map→base_link TF 없음: {type(e).__name__}). SLAM 이 켜져 있는지 확인')
            return
        if self.points:
            _, px, py = self.points[-1]
            if math.hypot(x - px, y - py) < self.min_gap:
                print(f'  [주의] 직전 지점과 {math.hypot(x - px, y - py):.2f} m 로 가깝습니다')
        name = name or f'지점{len(self.points) + 1}'
        self.points.append((name, x, y))
        self.save()
        print(f'  기록 {len(self.points)}: {name} ({x:+.2f}, {y:+.2f}) → {self.output}')

    def save(self):
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, 'w', encoding='utf-8') as f:
            f.write(route_yaml(self.points))

    def console(self):
        print('Enter = 현재 위치 기록 | 이름 입력 후 Enter = 그 이름으로 기록 | u = 마지막 지우기 | q = 끝내기')
        while rclpy.ok():
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if line == 'q':
                break
            if line == 'u':
                if self.points:
                    print(f'  지움: {self.points.pop()[0]}')
                    self.save()
                continue
            self.record(line)
        print(f'순찰 지점 {len(self.points)}개 저장: {self.output}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = RouteRecorder()
    threading.Thread(target=node.console, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
