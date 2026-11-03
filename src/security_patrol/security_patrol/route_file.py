"""순찰 경로 파일(patrol_route.yaml 형식) 만들기. ROS 없이 시험할 수 있게 따로 둔다."""


def route_yaml(points, dwell_time=2.0):
    """기록한 지점 목록을 patrol_node 가 읽는 yaml 문자열로 만든다. points: [(이름, x, y), ...]"""
    lines = ['# 8단계 실제 공간 순찰 경로 (route_recorder 로 기록). 좌표는 map 프레임 [m]',
             'patrol:',
             f'  dwell_time: {dwell_time:.1f}',
             '  checkpoints:']
    for name, x, y in points:
        safe = str(name).replace('\\', '').replace('"', '')
        lines.append(f'    - {{name: "{safe}", x: {x:.2f}, y: {y:.2f}}}')
    return '\n'.join(lines) + '\n'
