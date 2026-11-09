"""관제 앱 한 번에 켜기: 로봇(시뮬레이션/실제) + Nav2 + 이상 감지·알림 + 앱 서버.

이것만 켜고 브라우저로 앱을 열면 코스 선택·자동 순찰·수동 주행·레이더를 모두 앱에서 한다.

  # 시뮬레이션 (Gazebo 화면·RViz 없이)
  ros2 launch security_patrol app.launch.py
  # 실제 로봇 (맥에서 tools/microros_udp_relay.py 를 먼저 켠다)
  ros2 launch security_patrol app.launch.py robot:=real
  # 확인용으로 RViz 도 보기 / 이상 감지 끄기
  ros2 launch security_patrol app.launch.py rviz:=true detect:=false

브라우저: 맥 http://192.168.64.2:8080 (가상머신 주소), 가상머신 안 http://localhost:8080
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg_share, name, args):
    # include 인자가 바깥 launch 설정을 덮어쓰지 않도록 scoped 그룹으로 감싼다
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', name)),
            launch_arguments=args.items()),
    ])


def setup(context):
    pkg_share = get_package_share_directory('security_patrol')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('sim', 'real'):
        raise RuntimeError(f"robot 은 sim 또는 real 이어야 합니다 (받은 값: {robot})")
    sim = robot == 'sim'
    rviz = LaunchConfiguration('rviz')

    if sim:
        bringup = include(pkg_share, 'nav.launch.py', {'rviz': rviz})
        detect_args = {'use_sim_time': 'true'}
        app_params = {'use_sim_time': True}
    else:
        bringup = include(pkg_share, 'real_nav.launch.py', {'rviz': rviz})
        real_route = os.path.expanduser('~/maps_real/patrol_route_real.yaml')
        # 실제 로봇은 회전이 느려 감지를 오래 쉬므로 사건을 30초까지 이어 붙인다 (8단계 결정)
        detect_args = {'use_sim_time': 'false', 'relink_seconds': '30.0', 'route_file': real_route}
        # 4륜이 미끄러져 회전이 명령의 약 25% 라 수동 주행 회전 상한을 올린다
        app_params = {'use_sim_time': False, 'max_angular': 1.8}

    detect = GroupAction(condition=IfCondition(LaunchConfiguration('detect')), actions=[
        include(pkg_share, 'detect.launch.py', detect_args)])
    app = Node(package='security_patrol', executable='app_server', name='app_server', output='screen',
               parameters=[app_params, {'port': LaunchConfiguration('port')}])
    return [bringup, detect, app]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim (시뮬레이션) 또는 real (실제 로봇)'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz 도 켤지 (앱만 쓰면 false)'),
        DeclareLaunchArgument('detect', default_value='true', description='이상 감지·사건 관리·텔레그램 알림도 켤지'),
        DeclareLaunchArgument('port', default_value='8080', description='앱 주소의 포트'),
        OpaqueFunction(function=setup),
    ])
