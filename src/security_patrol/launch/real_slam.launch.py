"""8단계: 실제 로봇으로 기준 지도를 만든다 (2단계의 실제 로봇판).

구성:
  real_bringup.launch.py : micro-ROS agent + robot_bridge + robot_state_publisher
  slam_toolbox           : /scan + odom TF → /map
  rviz2 (slam_view.rviz) : 지도가 그려지는 과정 확인

로봇을 놓은 자리와 방향이 지도의 원점(0, 0, 0°)이 된다. 순찰할 때도 같은 자리에서 출발하면 편하다.

사용 예:
  ros2 launch security_patrol real_slam.launch.py
  # 다른 터미널: 키보드로 천천히 조종 (z 키로 속도를 더 줄일 수 있음)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.6
  # 다 돌았으면 지도 저장 (저장소 밖 ~/maps_real 에 둔다)
  mkdir -p ~/maps_real && ros2 run nav2_map_server map_saver_cli -f ~/maps_real/real_area
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml'),
            description='slam_toolbox 설정 파일 (시뮬레이션과 같은 파일, 시각만 실제 시계로 바꿈)'),
        DeclareLaunchArgument('agent', default_value='true', description='micro-ROS agent 를 함께 켤지'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),

        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'real_bringup.launch.py')),
                launch_arguments={'rviz': 'false', 'agent': LaunchConfiguration('agent')}.items()),
        ]),

        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_params_file'), {'use_sim_time': False}]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam_view.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
