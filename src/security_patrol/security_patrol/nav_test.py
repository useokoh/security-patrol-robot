"""Nav2 자동 목표 주행 테스트: 지도 위 여러 목표 지점으로 차례로 보내고 성공/실패와 걸린 시간을 출력한다.

- AMCL 초기 위치는 nav2_params.yaml 의 initial_pose 로 자동 설정된다 (시뮬레이션 시작점 -3, 0).
- 목표 지점은 장애물 반대편이나 칸막이 너머처럼 경로 계획이 필요한 곳으로 골랐다.
- Nav2 의 '도착'은 AMCL 추정 위치 기준이라, 위치를 잘못 알면 엉뚱한 곳에서도 성공으로 나온다.
  그래서 시뮬레이션에서는 /odom(= Gazebo 실제 위치)으로 실제로 목표에 왔는지도 확인한다 (check_truth).
  실제 로봇은 odom 에 오차가 쌓이므로 check_truth:=false 로 끈다.

  ros2 run security_patrol nav_test --ros-args -p use_sim_time:=true
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

# (이름, x, y, yaw[deg])  map 좌표 = 시뮬레이션 월드 좌표
GOALS = [
    ('기둥 너머 북쪽', -2.0, 2.2, 0.0),
    ('칸막이 너머 남동 구역', 1.6, -2.3, 0.0),
    ('북동쪽 상자 옆', 3.4, 0.9, 90.0),
    ('출발점 복귀', -3.0, 0.0, 0.0),
]
GOAL_TIMEOUT = 180.0  # 초 (시뮬레이션 시간)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(nav, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator()
    log = nav.get_logger()
    check_truth = nav.declare_parameter('check_truth', True).value
    truth_xy_tol = nav.declare_parameter('truth_xy_tol', 0.3).value       # [m]
    truth_yaw_tol = nav.declare_parameter('truth_yaw_tol_deg', 20.0).value  # [deg]

    # 실제 위치 (시뮬레이션의 /odom). isTaskComplete() 가 노드를 spin 하므로 콜백이 계속 갱신된다.
    truth = {'odom': None}
    nav.create_subscription(Odometry, 'odom', lambda m: truth.update(odom=m), qos_profile_sensor_data)

    # 주의: BasicNavigator.waitUntilNav2Active() 는 /amcl_pose 를 아직 못 받았으면
    # 초기 위치를 (0, 0, 0) 으로 '발행'해서 AMCL 을 엉뚱한 곳으로 리셋해버린다.
    # 그래서 먼저 AMCL 이 스스로 위치를 낼 때까지 기다린다 (시뮬레이션: nav2_params 의 initial_pose,
    # 실제 로봇: RViz 2D Pose Estimate). 받은 뒤에는 waitUntilNav2Active 가 초기 위치를 발행하지 않는다.
    log.info('AMCL 위치(/amcl_pose) 기다리는 중...')
    while not nav.initial_pose_received:
        rclpy.spin_once(nav, timeout_sec=0.5)
    log.info('Nav2 활성화 대기 중 (AMCL + bt_navigator)...')
    nav.waitUntilNav2Active()
    log.info('Nav2 준비 완료, 목표 주행 시작')

    results = []
    for name, x, y, yaw in GOALS:
        log.info(f'▶ {name} ({x:+.1f}, {y:+.1f}) 로 이동')
        nav.goToPose(make_pose(nav, x, y, yaw))
        start = nav.get_clock().now()
        wall_start = time.time()
        while not nav.isTaskComplete():
            elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > GOAL_TIMEOUT:
                log.warn('  시간 초과 → 목표 취소')
                nav.cancelTask()
            time.sleep(0.2)
        result = nav.getResult()
        elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
        ok = result == TaskResult.SUCCEEDED

        # 실제로 목표에 왔는지 확인 (최신 odom 을 받기 위해 잠깐 spin)
        truth_msg = ''
        if check_truth:
            for _ in range(5):
                rclpy.spin_once(nav, timeout_sec=0.1)
            odom = truth['odom']
            if odom is None:
                truth_msg = ' | 실제 위치: /odom 없음 (확인 생략)'
            else:
                p = odom.pose.pose
                err_xy = math.hypot(p.position.x - x, p.position.y - y)
                d = yaw_from_quaternion(p.orientation) - math.radians(yaw)
                err_yaw = abs(math.degrees(math.atan2(math.sin(d), math.cos(d))))
                truth_ok = err_xy <= truth_xy_tol and err_yaw <= truth_yaw_tol
                truth_msg = (f' | 실제 위치 오차 {err_xy:.2f} m / {err_yaw:.0f}° '
                             f'{"OK" if truth_ok else "→ 목표에서 벗어남"}')
                ok = ok and truth_ok

        results.append(ok)
        log.info(f'  {"PASS" if ok else "FAIL"} {name}: {result.name} '
                 f'(시뮬레이션 {elapsed:.0f}초 / 실제 {time.time() - wall_start:.0f}초){truth_msg}')

    passed = sum(results)
    log.info(f'=== 결과: {passed}/{len(results)} 통과 → '
             f'{"ALL PASS" if passed == len(results) else "FAIL 있음"} ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
