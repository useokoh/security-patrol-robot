"""4단계 순찰 노드: patrol_route.yaml 의 지점들을 Nav2 로 차례로 돌며 반복 순찰한다.

- 각 지점에 도착하면 dwell_time 동안 멈춰서 주변을 본다 (360° 라이다라 회전은 하지 않음).
- 지점에 못 가면 건너뛰고 다음 지점으로 간다. max_failures 번 연속 실패하면 순찰을 멈춘다.
- 현재 상태를 /patrol/status (std_msgs/String, JSON) 로 발행한다 → 6단계 경보에서 사용.
- laps:=0 이면 Ctrl+C 까지 계속 순찰, laps:=N 이면 N 바퀴 후 결과를 출력하고 종료한다.
- check_truth:=true (시뮬레이션 테스트용) 이면 /odom(= Gazebo 실제 위치)으로 실제 도착 여부도 확인한다.

  # 순찰 (계속 반복)
  ros2 run security_patrol patrol_node --ros-args -p use_sim_time:=true
  # 자동 테스트: 2바퀴 + 실제 위치 확인
  ros2 run security_patrol patrol_node --ros-args -p use_sim_time:=true -p laps:=2 -p check_truth:=true
"""

import json
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_route(path):
    """patrol_route.yaml 을 읽어 (dwell_time, [{name, x, y, yaw[rad]}...]) 로 돌려준다."""
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['patrol']
    points = cfg['checkpoints']
    route = []
    for i, p in enumerate(points):
        if 'yaw' in p:
            yaw = math.radians(float(p['yaw']))
        else:
            # yaw 생략 시 다음 지점(마지막이면 첫 지점)을 바라보는 방향
            nxt = points[(i + 1) % len(points)]
            yaw = math.atan2(nxt['y'] - p['y'], nxt['x'] - p['x'])
        route.append({'name': str(p['name']), 'x': float(p['x']), 'y': float(p['y']), 'yaw': yaw})
    return float(cfg.get('dwell_time', 2.0)), route


class Patrol:

    def __init__(self):
        self.nav = BasicNavigator(node_name='patrol_node')
        nav = self.nav
        default_route = os.path.join(
            get_package_share_directory('security_patrol'), 'config', 'patrol_route.yaml')
        self.route_file = nav.declare_parameter('route_file', default_route).value
        self.laps = int(nav.declare_parameter('laps', 0).value)                  # 0 = 무한 반복
        self.max_failures = int(nav.declare_parameter('max_failures', 3).value)  # 연속 실패 허용 횟수
        self.goal_timeout = float(nav.declare_parameter('goal_timeout', 120.0).value)  # [초, 시뮬레이션]
        self.check_truth = bool(nav.declare_parameter('check_truth', False).value)
        self.truth_xy_tol = float(nav.declare_parameter('truth_xy_tol', 0.3).value)
        self.dwell_time, self.route = load_route(self.route_file)

        self.status_pub = nav.create_publisher(String, 'patrol/status', 10)
        self.odom = None
        if self.check_truth:
            nav.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        self.odom = msg

    def publish_status(self, state, lap, idx, extra=None):
        cp = self.route[idx]
        data = {'state': state, 'lap': lap, 'index': idx, 'name': cp['name'],
                'x': cp['x'], 'y': cp['y']}
        if extra:
            data.update(extra)
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def sim_sleep(self, seconds):
        """시뮬레이션 시간 기준으로 기다린다 (그동안 콜백 처리)."""
        start = self.nav.get_clock().now()
        while (self.nav.get_clock().now() - start).nanoseconds / 1e9 < seconds:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def make_pose(self, cp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = cp['x']
        pose.pose.position.y = cp['y']
        pose.pose.orientation.z = math.sin(cp['yaw'] / 2.0)
        pose.pose.orientation.w = math.cos(cp['yaw'] / 2.0)
        return pose

    def go_to(self, lap, idx):
        """한 지점으로 이동. (성공 여부, 로그용 설명) 을 돌려준다."""
        nav = self.nav
        cp = self.route[idx]
        self.publish_status('moving', lap, idx)
        nav.goToPose(self.make_pose(cp))
        start = nav.get_clock().now()
        while not nav.isTaskComplete():
            if (nav.get_clock().now() - start).nanoseconds / 1e9 > self.goal_timeout:
                nav.get_logger().warn('  시간 초과 → 목표 취소')
                nav.cancelTask()
            time.sleep(0.1)
        elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
        result = nav.getResult()
        ok = result == TaskResult.SUCCEEDED
        info = f'{result.name}, {elapsed:.0f}초'

        if self.check_truth:
            for _ in range(5):
                rclpy.spin_once(nav, timeout_sec=0.1)
            if self.odom is None:
                info += ' | 실제 위치: /odom 없음'
            else:
                p = self.odom.pose.pose.position
                err = math.hypot(p.x - cp['x'], p.y - cp['y'])
                truth_ok = err <= self.truth_xy_tol
                info += f' | 실제 위치 오차 {err:.2f} m {"OK" if truth_ok else "→ 벗어남"}'
                ok = ok and truth_ok
        return ok, info

    def run(self):
        nav = self.nav
        log = nav.get_logger()
        log.info(f'순찰 경로: {len(self.route)}개 지점, 정지 {self.dwell_time:.0f}초, '
                 f'{"무한 반복" if self.laps == 0 else f"{self.laps}바퀴"} ({self.route_file})')
        # 주의: BasicNavigator.waitUntilNav2Active() 는 /amcl_pose 를 아직 못 받았으면
        # 초기 위치를 (0, 0, 0) 으로 '발행'해서 AMCL 을 엉뚱한 곳으로 리셋해버린다.
        # 그래서 먼저 AMCL 이 스스로 위치를 낼 때까지 기다린다 (시뮬레이션: nav2_params 의 initial_pose,
        # 실제 로봇: RViz 2D Pose Estimate). 받은 뒤에는 waitUntilNav2Active 가 초기 위치를 발행하지 않는다.
        log.info('AMCL 위치(/amcl_pose) 기다리는 중...')
        while not nav.initial_pose_received:
            rclpy.spin_once(nav, timeout_sec=0.5)
        log.info('Nav2 활성화 대기 중...')
        nav.waitUntilNav2Active()

        results = []          # (바퀴, 지점 이름, 성공 여부)
        failures = 0          # 연속 실패 횟수
        lap = 1
        while self.laps == 0 or lap <= self.laps:
            lap_start = nav.get_clock().now()
            log.info(f'===== {lap}바퀴째 순찰 시작 =====')
            for idx, cp in enumerate(self.route):
                log.info(f'▶ [{lap}-{idx + 1}] {cp["name"]} ({cp["x"]:+.2f}, {cp["y"]:+.2f})')
                ok, info = self.go_to(lap, idx)
                results.append((lap, cp['name'], ok))
                if ok:
                    failures = 0
                    log.info(f'  도착 ({info}) → {self.dwell_time:.0f}초 감시')
                    self.publish_status('watching', lap, idx)
                    self.sim_sleep(self.dwell_time)
                else:
                    failures += 1
                    log.warn(f'  실패 ({info}) → 다음 지점으로 건너뜀 (연속 {failures}회)')
                    self.publish_status('skipped', lap, idx, {'failures': failures})
                    if failures >= self.max_failures:
                        log.error(f'!!! {failures}회 연속 실패 → 순찰 중지. 로봇 상태를 확인하세요')
                        self.publish_status('stopped', lap, idx, {'failures': failures})
                        return self.report(results, aborted=True)
            lap_time = (nav.get_clock().now() - lap_start).nanoseconds / 1e9
            log.info(f'===== {lap}바퀴 완료 ({lap_time:.0f}초) =====')
            lap += 1
        return self.report(results, aborted=False)

    def report(self, results, aborted):
        passed = sum(ok for _, _, ok in results)
        failed = [f'{lap}-{name}' for lap, name, ok in results if not ok]
        all_ok = not aborted and not failed
        self.nav.get_logger().info(
            f'=== 결과: {passed}/{len(results)} 지점 성공'
            f'{" (실패: " + ", ".join(failed) + ")" if failed else ""}'
            f'{" / 중도 중지" if aborted else ""} → {"ALL PASS" if all_ok else "FAIL 있음"} ===')
        return all_ok


def main(args=None):
    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 끊어버려서 Nav2 에 '이동 취소'를 보낼 수 없다.
    # 기본 처리를 끄고 KeyboardInterrupt 에서 직접 취소한 뒤 종료한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    patrol = Patrol()
    try:
        patrol.run()
    except KeyboardInterrupt:
        # 진행 중인 이동을 취소하지 않으면 Nav2 가 현재 목표까지 계속 주행한다
        patrol.nav.get_logger().info('순찰 중단 (Ctrl+C) → 이동 취소')
        patrol.nav.cancelTask()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
