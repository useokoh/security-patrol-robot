"""5-2 시험 도구: Gazebo 안의 침입자 모델을 직선으로 왕복시켜 '움직이는 대상'을 만든다.

먼저 README 의 명령으로 침입자(intruder1)를 만든 뒤 실행한다.

  ros2 run security_patrol intruder_walker
  ros2 run security_patrol intruder_walker --speed 0.3 --duration 60

Gazebo 의 set_pose 서비스를 ign 명령으로 호출한다. 호출이 느리면 한 번에 많이 건너뛰어
감지 노드가 같은 물체로 잇지 못하므로, 끝날 때 호출 간격을 출력한다 (0.5 m/s 면 0.8초 미만이어야 함).
"""

import argparse
import subprocess
import time


def position_at(elapsed, start, end, speed):
    """시작점에서 끝점까지 왕복할 때 elapsed 초 시점의 위치를 돌려준다."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = (dx * dx + dy * dy) ** 0.5
    travelled = (elapsed * speed) % (2.0 * length)
    if travelled > length:
        travelled = 2.0 * length - travelled
    ratio = travelled / length
    return start[0] + dx * ratio, start[1] + dy * ratio


def set_pose(world, name, x, y):
    """Gazebo 모델 위치를 옮긴다. 성공하면 True."""
    request = f'name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}'
    result = subprocess.run(
        ['ign', 'service', '-s', f'/world/{world}/set_pose',
         '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
         '--timeout', '1000', '--req', request],
        capture_output=True, text=True)
    return result.returncode == 0 and 'true' in result.stdout


def main():
    parser = argparse.ArgumentParser(description='침입자 모델을 직선으로 왕복시킨다')
    parser.add_argument('--world', default='restricted_area')
    parser.add_argument('--name', default='intruder1')
    parser.add_argument('--start', type=float, nargs=2, default=[-1.0, 0.3], metavar=('X', 'Y'))
    parser.add_argument('--end', type=float, nargs=2, default=[0.4, 0.3], metavar=('X', 'Y'))
    parser.add_argument('--speed', type=float, default=0.5, help='걷는 속도 [m/s]')
    parser.add_argument('--duration', type=float, default=120.0, help='실행 시간 [초]')
    args, _ = parser.parse_known_args()

    print(f'{args.name}: {args.start} ↔ {args.end}, {args.speed} m/s, {args.duration:.0f}초')
    begin = time.monotonic()
    calls, failures, slowest = 0, 0, 0.0
    try:
        while time.monotonic() - begin < args.duration:
            call_start = time.monotonic()
            x, y = position_at(call_start - begin, args.start, args.end, args.speed)
            if not set_pose(args.world, args.name, x, y):
                failures += 1
            calls += 1
            taken = time.monotonic() - call_start
            slowest = max(slowest, taken)
            time.sleep(max(0.0, 0.2 - taken))   # 최대 초당 5번
    except KeyboardInterrupt:
        pass
    if calls:
        print(f'호출 {calls}번, 실패 {failures}번, 평균 간격 {(time.monotonic() - begin) / calls:.2f}초, '
              f'가장 느린 호출 {slowest:.2f}초')


if __name__ == '__main__':
    main()
