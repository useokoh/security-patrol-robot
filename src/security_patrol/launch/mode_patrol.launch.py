"""순찰 모드: 기준 지도(map_server) + AMCL 위치추정 + Nav2 + 이상 감지·알림 (+ RViz 선택).

로봇 바탕(robot_base.launch.py)이 켜져 있어야 한다. 보통은 앱 서버가 켜고 끈다.

  ros2 launch security_patrol mode_patrol.launch.py robot:=sim map:=<지도 yaml>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, args):
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())])


def setup(context):
    pkg_share = get_package_share_directory('security_patrol')
    nav2_share = get_package_share_directory('nav2_bringup')
    robot = LaunchConfiguration('robot').perform(context)
    sim = robot == 'sim'
    if not sim and robot != 'real':
        raise RuntimeError(f'robot 은 sim 또는 real 이어야 합니다 (받은 값: {robot})')
    default_map = (os.path.join(pkg_share, 'maps', 'restricted_area.yaml') if sim
                   else os.path.expanduser('~/maps_real/real_area.yaml'))
    map_file = os.path.expanduser(LaunchConfiguration('map').perform(context) or default_map)
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml' if sim else 'nav2_params_real.yaml')
    use_sim = 'true' if sim else 'false'

    actions = [include(os.path.join(nav2_share, 'launch', 'bringup_launch.py'), {
        'map': map_file, 'params_file': params, 'use_sim_time': use_sim, 'autostart': 'true'})]
    if LaunchConfiguration('detect').perform(context).lower() == 'true':
        detect_args = {'use_sim_time': use_sim}
        if not sim:   # 실제 로봇은 회전이 느려 사건을 30초까지 이어 붙인다 (8단계 결정)
            detect_args.update(relink_seconds='30.0', route_file=os.path.expanduser('~/maps_real/patrol_route_real.yaml'))
        actions.append(include(os.path.join(pkg_share, 'launch', 'detect.launch.py'), detect_args))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': sim}], condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim 또는 real'),
        DeclareLaunchArgument('map', default_value='', description='기준 지도 yaml (비우면 기본 지도)'),
        DeclareLaunchArgument('detect', default_value='true', description='이상 감지·알림도 켤지'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz 도 켤지'),
        OpaqueFunction(function=setup),
    ])
