"""3단계: 저장된 기준 지도 위에서 위치추정(AMCL) + 자율주행(Nav2).

구성:
  sim_world.launch.py (Gazebo + 브리지 + robot_state_publisher, 자체 RViz 는 끔)
  nav2_bringup/bringup_launch.py : map_server + amcl + Nav2 전체 (planner, controller, behavior, bt_navigator ...)
  rviz2 (nav2_default_view.rviz) : 2D Pose Estimate / Nav2 Goal 도구 포함

사용 예:
  ros2 launch security_patrol nav.launch.py            # Gazebo 화면을 보려면 gui:=true
  # 다른 터미널: 위치추정 점검 (AMCL 추정 vs 실제 위치)
  ros2 run security_patrol loc_monitor --ros-args -p use_sim_time:=true
  # 또 다른 터미널: 자동 목표 주행 테스트
  ros2 run security_patrol nav_test --ros-args -p use_sim_time:=true
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
            'map', default_value=os.path.join(pkg_share, 'maps', 'restricted_area.yaml'),
            description='기준 지도 yaml'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 설정 파일'),
        # 가상머신에서는 Gazebo 화면이 CPU 를 가장 많이 써서 기본으로 끈다 (로봇 상태는 RViz 로 확인)
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo 화면 표시 여부'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),

        # include 인자가 바깥 launch 설정을 덮어쓰지 않도록 각각 scoped 그룹으로 감싼다
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'sim_world.launch.py')),
                launch_arguments={'rviz': 'false', 'gui': LaunchConfiguration('gui')}.items()),
        ]),

        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items()),
        ]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
