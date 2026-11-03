"""8단계: 실제 로봇 연결 (Gazebo 대신).

구성:
  micro_ros_agent (UDP 8888)  : ESP32 와 ROS2 를 잇는다. 맥의 microros_udp_relay.py 가 로봇 패킷을 여기로 넘겨 준다
  robot_bridge                : ESP32 원시 데이터 → /scan, /odom, odom→base_link TF, /joint_states
  robot_state_publisher       : URDF(use_sim:=false) → base_link → laser_frame 등 로봇 내부 TF
  rviz2 (선택)                : 라이다와 TF 확인

사용 예:
  ros2 launch security_patrol real_bringup.launch.py rviz:=true
  # 다른 터미널: 연결 점검 (scan, odom 이 들어오는지)
  ros2 run security_patrol scan_check
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('security_patrol')
    urdf = os.path.join(pkg_share, 'urdf', 'patrol_bot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value='true',
                              description='micro-ROS agent 를 함께 켤지 (따로 켰으면 false)'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz 실행 여부'),

        Node(
            package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
            output='screen', arguments=['udp4', '--port', '8888'],
            condition=IfCondition(LaunchConfiguration('agent'))),

        Node(
            package='security_patrol', executable='robot_bridge', name='robot_bridge', output='screen',
            parameters=[{'use_sim_time': False}]),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'sim_view.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
