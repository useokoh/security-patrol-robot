"""순찰 경로 파일(patrol_route.yaml 형식) 만들기. ROS 없이 시험할 수 있게 따로 둔다."""


def clean(text):
    """yaml 큰따옴표 안에 넣어도 깨지지 않게 \\ 와 " 를 뺀다."""
    return str(text).replace('\\', '').replace('"', '').replace('\n', ' ')


def route_yaml(points, dwell_time=2.0, name=None,
               header='8단계 실제 공간 순찰 경로 (route_recorder 로 기록)'):
    """지점 목록을 patrol_node 가 읽는 yaml 문자열로 만든다. points: [(이름, x, y), ...]

    name 을 주면 관제 앱 코스 목록에 그 이름으로 보인다 (patrol_node 는 읽지 않는다).
    """
    lines = [f'# {clean(header)}. 좌표는 map 프레임 [m]', 'patrol:']
    if name:
        lines.append(f'  name: "{clean(name)}"')
    lines += [f'  dwell_time: {dwell_time:.1f}', '  checkpoints:']
    for name_, x, y in points:
        safe = clean(name_)
        lines.append(f'    - {{name: "{safe}", x: {x:.2f}, y: {y:.2f}}}')
    return '\n'.join(lines) + '\n'
