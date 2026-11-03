"""8단계: 실제 로봇 + 저장한 실제 지도 위 위치추정(AMCL) + 자율주행(Nav2) (3단계의 실제 로봇판).

구성:
  real_bringup.launch.py         : micro-ROS agent + robot_bridge + robot_state_publisher
  nav2_bringup/bringup_launch.py : map_server + amcl + Nav2 (설정: config/nav2_params_real.yaml)
  rviz2 (nav2_default_view.rviz) : 2D Pose Estimate / Nav2 Goal 도구 포함

AMCL 시작 위치는 지도 원점(지도를 만들 때 출발한 자리)이다. 다른 곳에 놓았으면 RViz 2D Pose Estimate 로 알려 준다.

사용 예:
  ros2 launch security_patrol real_nav.launch.py                    # 지도 기본값 ~/maps_real/real_area.yaml
  ros2 launch security_patrol real_nav.launch.py map:=/경로/지도.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('security_patrol')
    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/maps_real/real_area.yaml'),
            description='실제 공간 기준 지도 yaml (저장소에 올리지 않도록 저장소 밖에 둔다)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params_real.yaml'),
            description='Nav2 설정 파일 (실제 로봇용)'),
        DeclareLaunchArgument('agent', default_value='true', description='micro-ROS agent 를 함께 켤지'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),

        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'real_bringup.launch.py')),
                launch_arguments={'rviz': 'false', 'agent': LaunchConfiguration('agent')}.items()),
        ]),

        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'false',
                    'autostart': 'true',
                }.items()),
        ]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
