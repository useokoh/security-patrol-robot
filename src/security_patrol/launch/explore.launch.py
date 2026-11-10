"""자율 탐사 한 번에 켜기 (앱 없이): 로봇 바탕 + 지도 만들기 모드(slam_toolbox + Nav2 이동 + explorer).

로봇이 스스로 돌아다니며 지도를 채우고, 끝나면 출발점으로 돌아와 지도와 순찰 코스를 저장한다.
앱에서는 '지도' 탭의 지도 만들기 버튼이 같은 일을 한다 (app.launch.py).

  # 시뮬레이션 (저장: ~/sentinel_maps/sim/explored.yaml, 코스 ~/sentinel_courses/sim/explored_course.yaml)
  ros2 launch security_patrol explore.launch.py
  # 실제 로봇 (맥에서 tools/microros_udp_relay.py 를 먼저 켬. ~/maps_real/explored_area.yaml 로 저장)
  ros2 launch security_patrol explore.launch.py robot:=real
  # 탐사 결과로 순찰하기 (시뮬레이션 예)
  ros2 launch security_patrol nav.launch.py map:=$HOME/sentinel_maps/sim/explored.yaml

탐사 지도의 원점은 탐사를 시작한 로봇 위치다. 순찰할 때도 같은 자리·방향에서 시작해야 코스 좌표가 맞는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(path, args):
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())])


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('security_patrol'), 'launch')
    robot = LaunchConfiguration('robot')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim (시뮬레이션) 또는 real (실제 로봇)'),
        DeclareLaunchArgument('rviz', default_value='false', description='지도가 채워지는 모습을 RViz 로도 볼지'),
        DeclareLaunchArgument('save_map', default_value='', description='지도 저장 경로 (확장자 없이, 비우면 기본값)'),
        DeclareLaunchArgument('course_file', default_value='', description='순찰 코스 저장 경로 (비우면 기본값)'),
        include(os.path.join(launch_dir, 'robot_base.launch.py'), {'robot': robot}),
        include(os.path.join(launch_dir, 'mode_explore.launch.py'), {
            'robot': robot, 'rviz': LaunchConfiguration('rviz'),
            'save_map': LaunchConfiguration('save_map'), 'course_file': LaunchConfiguration('course_file')}),
    ])
