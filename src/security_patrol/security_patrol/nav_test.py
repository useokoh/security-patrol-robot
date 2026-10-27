"""Nav2 자동 목표 주행 테스트: 지도 위 여러 목표 지점으로 차례로 보내고 성공/실패와 걸린 시간을 출력한다.

- AMCL 초기 위치는 nav2_params.yaml 의 initial_pose 로 자동 설정된다 (시뮬레이션 시작점 -3, 0).
- 목표 지점은 장애물 반대편이나 칸막이 너머처럼 경로 계획이 필요한 곳으로 골랐다.

  ros2 run security_patrol nav_test --ros-args -p use_sim_time:=true
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (이름, x, y, yaw[deg])  map 좌표 = 시뮬레이션 월드 좌표
GOALS = [
    ('기둥 너머 북쪽', -2.0, 2.2, 0.0),
    ('칸막이 너머 남동 구역', 1.6, -2.3, 0.0),
    ('북동쪽 상자 옆', 3.4, 0.9, 90.0),
    ('출발점 복귀', -3.0, 0.0, 0.0),
]
GOAL_TIMEOUT = 180.0  # 초 (시뮬레이션 시간)


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
        results.append(ok)
        log.info(f'  {"PASS" if ok else "FAIL"} {name}: {result.name} '
                 f'(시뮬레이션 {elapsed:.0f}초 / 실제 {time.time() - wall_start:.0f}초)')

    passed = sum(results)
    log.info(f'=== 결과: {passed}/{len(results)} 통과 → '
             f'{"ALL PASS" if passed == len(results) else "FAIL 있음"} ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
