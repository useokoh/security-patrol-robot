"""자율 탐사 노드: 로봇이 스스로 돌아다니며 지도를 채우고, 끝나면 지도와 순찰 코스까지 만든다.

사람이 키보드로 몰며 지도를 만들고(real_slam) route_recorder 로 지점을 찍던 일을 대신한다.
slam_toolbox 가 /map 을 계속 넓히는 동안, 이 노드가 프런티어(아는 곳과 모르는 곳의 경계)를 골라
Nav2 로 보낸다 (explore_logic.choose_goal). 경계가 더 없으면:
  1. 출발점으로 돌아온다 (return_home)
  2. 지도를 저장한다 (save_map: 경로를 주면 map_saver_cli 로 .pgm/.yaml)
  3. 그 지도로 순찰 코스를 만든다 (course_file: course_planner, 관제 앱 코스 목록에 보인다)
진행 상황은 /explore/status (std_msgs/String, JSON) 로 낸다.

  # 보통은 explore.launch.py 로 시뮬레이션/실제 로봇 + SLAM + Nav2 와 함께 켠다
  ros2 launch security_patrol explore.launch.py
"""

import json
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from security_patrol.course_planner import plan_course
from security_patrol.explore_logic import choose_goal, frontier_near, known_area
from security_patrol.route_file import route_yaml


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorer:

    def __init__(self):
        self.nav = BasicNavigator(node_name='explorer')
        nav = self.nav
        p = nav.declare_parameter
        self.goal_timeout = float(p('goal_timeout', 60.0).value)     # 한 목표에 쓰는 최대 시간 [초]
        self.check_period = float(p('check_period', 2.0).value)      # 가는 도중 경계가 이미 보였는지 확인 [초]
        self.max_goals = int(p('max_goals', 60).value)               # 무한 반복 방지
        self.return_home = bool(p('return_home', True).value)
        self.save_map = os.path.expanduser(str(p('save_map', '').value))        # '' = 저장 안 함
        self.course_file = os.path.expanduser(str(p('course_file', '').value))  # '' = 코스 안 만듦
        self.course_name = str(p('course_name', '탐사로 만든 코스').value)
        self.min_size = float(p('min_frontier', 0.3).value)          # 이보다 짧은 경계는 무시 [m]
        self.max_radius = float(p('max_radius', 0.0).value)          # 출발점에서 이보다 먼 곳은 안 감 [m], 0 = 제한 없음
        self.map = None
        self.blacklist = []             # 못 간 곳·볼 수 없는 경계 (다시 고르지 않음)
        self.visited = []               # 가 본 관찰 지점 (두 곳 사이 왕복 방지)
        self.stats = {'goals': 0, 'failed': 0, 'abandoned': 0, 'started': time.time()}
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        nav.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.status_pub = nav.create_publisher(String, 'explore/status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, nav)

    # ---------- 입력 ----------

    def on_map(self, msg):
        i = msg.info
        self.map = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, list(msg.data))

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def publish(self, state, **extra):
        area = known_area(self.map[5], self.map[2]) if self.map else 0.0
        data = dict(state=state, known_area=round(area, 2), goals=self.stats['goals'],
                    failed=self.stats['failed'], abandoned=self.stats['abandoned'],
                    elapsed=round(time.time() - self.stats['started']), **extra)
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.nav.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    # ---------- 탐사 ----------

    def go(self, x, y, yaw, watch=False):
        """목표로 이동. watch 면 가는 도중 목표 1 m 안 경계가 이미 다 보였는지 보고 일찍 끝낸다.

        (관찰 지점은 경계에서 0.8 m 안에서 고르므로, 1 m 안에 경계가 없으면 그 경계는 이미 밝혀진 것)
        결과: 'ok' / 'seen' / 'failed'.
        """
        nav = self.nav
        nav.goToPose(self.make_pose(x, y, yaw))
        start = last_check = time.time()
        while not nav.isTaskComplete():
            now = time.time()
            if now - start > self.goal_timeout:
                nav.get_logger().warn('  시간 초과 → 목표 취소')
                nav.cancelTask()
                return 'failed'
            if watch and now - last_check > self.check_period:
                last_check = now
                if not frontier_near(*self.map, (x, y), 1.0, self.min_size):
                    nav.cancelTask()                         # 이미 보였으니 끝까지 갈 필요 없다
                    return 'seen'
            time.sleep(0.1)
        return 'ok' if nav.getResult() == TaskResult.SUCCEEDED else 'failed'

    def run(self):
        nav, log = self.nav, self.nav.get_logger()
        self.publish('starting')
        log.info('지도(/map)와 로봇 위치(map→base_link) 기다리는 중...')
        while rclpy.ok() and (self.map is None or self.pose() is None):
            rclpy.spin_once(nav, timeout_sec=0.5)
        # SLAM 모드라 AMCL 이 없다. localizer 로 controller_server 를 주면 초기 위치를 기다리지 않는다
        log.info('Nav2 활성화 대기 중...')
        nav.waitUntilNav2Active(localizer='controller_server')
        home = self.pose()
        log.info(f'탐사 시작 (출발점 {home[0]:+.2f}, {home[1]:+.2f}'
                 f'{f", 범위 {self.max_radius:.1f} m" if self.max_radius > 0 else ""})')

        while rclpy.ok() and self.stats['goals'] < self.max_goals:
            self.spin_for(0.5)                               # 최신 지도 받기
            robot = self.pose()
            if robot is None:
                continue
            pick = choose_goal(*self.map, robot[:2], self.blacklist, self.visited, min_size=self.min_size,
                               home=home[:2], max_radius=self.max_radius)
            if pick['abandon']:
                self.blacklist += pick['abandon']
                self.stats['abandoned'] += len(pick['abandon'])
                log.info(f'  가서도 안 보이는 경계 포기: {pick["abandon"]}')
            if pick['goal'] is None:
                break
            gx, gy = pick['goal']
            lx, ly = pick['look']
            yaw = math.atan2(ly - gy, lx - gx)                # 경계를 바라보며 도착
            self.stats['goals'] += 1
            log.info(f'▶ [{self.stats["goals"]}] 목표 ({gx:+.2f}, {gy:+.2f}) 경계 {pick["size"]:.2f} m, '
                     f'이동 {pick["distance"]:.1f} m, 남은 경계 {pick["frontiers"]}개')
            self.publish('exploring', goal=[gx, gy], frontiers=pick['frontiers'])
            result = self.go(gx, gy, yaw, watch=True)
            if result == 'failed':
                self.stats['failed'] += 1
                self.blacklist.append((gx, gy))
                log.warn('  못 감 → 이 목표는 다시 고르지 않음')
            else:
                self.visited.append((gx, gy))
                log.info('  도착' if result == 'ok' else '  가는 도중 경계가 보여서 다음 목표로')

        area = known_area(self.map[5], self.map[2])
        log.info(f'=== 탐사 끝: 목표 {self.stats["goals"]}개 (실패 {self.stats["failed"]}, 포기 {self.stats["abandoned"]}), '
                 f'아는 빈 공간 {area:.1f} m², {time.time() - self.stats["started"]:.0f}초 ===')
        home_ok = None
        if self.return_home:
            # 좁은 곳에서 끝나면 한 번에 못 나올 수 있다 → 세 번까지 (사이에 Nav2 복구 동작이 돈다)
            self.publish('returning')
            for attempt in range(1, 4):
                log.info(f'출발점으로 돌아가는 중... ({attempt}/3)')
                home_ok = self.go(home[0], home[1], home[2]) == 'ok'
                if home_ok:
                    break
            if not home_ok:
                log.warn('!!! 출발점으로 돌아오지 못함. 순찰 모드로 바꿀 때 앱 서버가 지금 위치로 위치추정을 맞춘다')
        self.spin_for(1.0)
        map_file = self.save_map_file()
        course_file = self.save_course(home)
        self.publish('done', map_file=map_file, course_file=course_file, home_ok=home_ok)

    def save_map_file(self):
        if not self.save_map:
            return None
        os.makedirs(os.path.dirname(self.save_map) or '.', exist_ok=True)
        self.publish('saving')
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.save_map,
               '--ros-args', '-p', f'use_sim_time:={"true" if self.nav.get_parameter("use_sim_time").value else "false"}']
        try:
            subprocess.run(cmd, timeout=30, check=True)
        except (OSError, subprocess.SubprocessError) as e:
            self.nav.get_logger().error(f'지도 저장 실패: {e}')
            return None
        self.nav.get_logger().info(f'지도 저장: {self.save_map}.yaml / .pgm')
        return self.save_map + '.yaml'

    def save_course(self, home):
        if not self.course_file:
            return None
        try:
            plan = plan_course(*self.map, home[:2])
        except ValueError as e:
            self.nav.get_logger().error(f'코스를 만들지 못함: {e}')
            return None
        os.makedirs(os.path.dirname(self.course_file) or '.', exist_ok=True)
        with open(self.course_file, 'w', encoding='utf-8') as f:
            f.write(route_yaml([(q['name'], q['x'], q['y']) for q in plan['points']], 2.0, name=self.course_name,
                               header='자율 탐사로 만든 지도에서 자동으로 만든 코스'))
        self.nav.get_logger().info(f'순찰 코스 저장: {self.course_file} (지점 {len(plan["points"])}개, '
                                   f'한 바퀴 {plan["length"]} m, 보이는 공간 {plan["coverage"] * 100:.0f}%)')
        return self.course_file


def main(args=None):
    # 기본 Ctrl+C 처리는 통신을 먼저 끊어 Nav2 에 '이동 취소'를 못 보낸다 (patrol_node 와 같은 방식)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    explorer = Explorer()
    try:
        explorer.run()
    except KeyboardInterrupt:
        explorer.nav.get_logger().info('탐사 중단 (Ctrl+C) → 이동 취소')
        explorer.nav.cancelTask()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
