"""지도 만들기 모드: slam_toolbox + Nav2(이동만) + explorer (+ RViz 선택).

로봇 바탕(robot_base.launch.py)이 켜져 있어야 한다. 보통은 앱 서버가 켜고 끈다.
기준 지도가 없으므로 map_server·AMCL 대신 slam_toolbox 가 /map 과 map→odom 을 만든다.

  ros2 launch security_patrol mode_explore.launch.py robot:=sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 탐사 결과 저장 위치 (실제 로봇은 기존 지도 real_area 를 덮어쓰지 않게 다른 이름)
DEFAULTS = {
    'sim': {'map': '~/sentinel_maps/sim/explored', 'course': '~/sentinel_courses/sim/explored_course.yaml'},
    'real': {'map': '~/maps_real/explored_area', 'course': '~/maps_real/patrol_route_explored.yaml'},
}


def setup(context):
    pkg_share = get_package_share_directory('security_patrol')
    nav2_share = get_package_share_directory('nav2_bringup')
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in DEFAULTS:
        raise RuntimeError(f'robot 은 sim 또는 real 이어야 합니다 (받은 값: {robot})')
    sim = robot == 'sim'
    save_map = LaunchConfiguration('save_map').perform(context) or DEFAULTS[robot]['map']
    course = LaunchConfiguration('course_file').perform(context) or DEFAULTS[robot]['course']
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml' if sim else 'nav2_params_real.yaml')
    return [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml'), {'use_sim_time': sim}]),
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'true' if sim else 'false',
                              'autostart': 'true'}.items())]),
        Node(package='security_patrol', executable='explorer', name='explorer', output='screen',
             parameters=[{'use_sim_time': sim, 'save_map': save_map, 'course_file': course,
                          'return_home': True, 'goal_timeout': 60.0 if sim else 90.0}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam_view.rviz')],
             parameters=[{'use_sim_time': sim}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim 또는 real'),
        DeclareLaunchArgument('save_map', default_value='', description='지도 저장 경로 (확장자 없이, 비우면 기본값)'),
        DeclareLaunchArgument('course_file', default_value='', description='순찰 코스 저장 경로 (비우면 기본값)'),
        DeclareLaunchArgument('rviz', default_value='false', description='지도가 채워지는 모습을 RViz 로도 볼지'),
        OpaqueFunction(function=setup),
    ])
