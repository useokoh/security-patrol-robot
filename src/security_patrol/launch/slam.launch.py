"""2단계: 시뮬레이션 + slam_toolbox 로 기준 지도를 만든다.

구성:
  sim_world.launch.py (Gazebo + 브리지 + robot_state_publisher, RViz 는 끔)
  slam_toolbox (online async) : /scan + odom TF → /map, map → odom TF
  rviz2 (slam_view.rviz)       : 지도가 그려지는 과정 확인

사용 예:
  ros2 launch security_patrol slam.launch.py
  # 다른 터미널: 자동 경로 주행 (또는 teleop_twist_keyboard 로 직접 조종)
  ros2 run security_patrol auto_mapper --ros-args -p use_sim_time:=true --params-file <mapping_route.yaml>
  # 주행이 끝나면 지도 저장
  ros2 run nav2_map_server map_saver_cli -f ~/security_patrol_ws/src/security_patrol/maps/restricted_area
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
            description='slam_toolbox 설정 파일'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 화면 표시 여부'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),

        # include 에 넘긴 인자는 범위가 나뉘지 않아 바깥 'rviz' 값까지 'false' 로 덮어쓴다.
        # scoped 그룹으로 감싸서 sim_world 의 RViz 만 끄고, 아래 SLAM 용 RViz 는 그대로 띄운다.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'sim_world.launch.py')),
                launch_arguments={'rviz': 'false', 'gui': LaunchConfiguration('gui')}.items()),
        ]),

        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_params_file'), {'use_sim_time': True}]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam_view.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
