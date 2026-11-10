"""관제 앱 한 번에 켜기: 로봇 바탕(시뮬레이션/실제) + 앱 서버.

앱 서버가 '모드'를 켜고 끈다. 처음에는 순찰 모드(기준 지도 + AMCL + Nav2 + 이상 감지)로 시작하고,
앱의 '지도' 탭에서 지도 만들기(자율 탐사)를 누르면 지도 만들기 모드(slam_toolbox + Nav2 + explorer)로 바꾼다.
그 뒤 코스 선택·자동 순찰·수동 주행·레이더를 모두 앱에서 한다.

  # 시뮬레이션 (Gazebo 화면·RViz 없이)
  ros2 launch security_patrol app.launch.py
  # 실제 로봇 (맥에서 tools/microros_udp_relay.py 를 먼저 켠다)
  ros2 launch security_patrol app.launch.py robot:=real
  # 확인용으로 RViz 도 보기 / 이상 감지 끄기 / 다른 기준 지도로 시작
  ros2 launch security_patrol app.launch.py rviz:=true detect:=false map:=$HOME/sentinel_maps/sim/explored.yaml

브라우저: 맥 http://192.168.64.2:8080 (가상머신 주소), 가상머신 안 http://localhost:8080
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    pkg_share = get_package_share_directory('security_patrol')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('sim', 'real'):
        raise RuntimeError(f'robot 은 sim 또는 real 이어야 합니다 (받은 값: {robot})')
    sim = robot == 'sim'
    base = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot_base.launch.py')),
        launch_arguments={'robot': robot}.items())])
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')
    # 값은 여기서 확정해 넘긴다 (빈 문자열을 그대로 파라미터로 넘기면 '값 없음'이 되어 노드가 실패할 수 있다)
    params = {'use_sim_time': sim, 'robot': robot, 'start_mode': 'patrol',
              'port': int(LaunchConfiguration('port').perform(context)),
              'detect': flag('detect'), 'rviz': flag('rviz')}
    map_file = LaunchConfiguration('map').perform(context)
    if map_file:
        params['map_file'] = os.path.expanduser(map_file)
    if not sim:   # 4륜이 미끄러져 회전이 명령의 약 25% 라 수동 주행 회전 상한을 올린다
        params['max_angular'] = 1.8
    app = Node(package='security_patrol', executable='app_server', name='app_server', output='screen',
               parameters=[params])
    return [base, app]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim (시뮬레이션) 또는 real (실제 로봇)'),
        DeclareLaunchArgument('map', default_value='', description='순찰 모드 기준 지도 yaml (비우면 기본 지도)'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz 도 켤지 (앱만 쓰면 false)'),
        DeclareLaunchArgument('detect', default_value='true', description='순찰 모드에서 이상 감지·알림도 켤지'),
        DeclareLaunchArgument('port', default_value='8080', description='앱 주소의 포트'),
        OpaqueFunction(function=setup),
    ])
