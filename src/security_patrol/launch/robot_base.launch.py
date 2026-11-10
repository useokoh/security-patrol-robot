"""로봇 바탕만 켜기: 시뮬레이션(Gazebo + 브리지) 또는 실제 로봇(micro-ROS agent + robot_bridge).

지도·위치추정·Nav2 는 켜지 않는다. app.launch.py 가 이것을 켜 두고, 앱 서버가 모드(순찰 / 지도 만들기)를
바꿔 가며 mode_patrol.launch.py / mode_explore.launch.py 를 켜고 끈다.

  ros2 launch security_patrol robot_base.launch.py robot:=sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    pkg_share = get_package_share_directory('security_patrol')
    robot = LaunchConfiguration('robot').perform(context)
    if robot == 'sim':
        name, args = 'sim_world.launch.py', {'rviz': 'false', 'gui': LaunchConfiguration('gui')}
    elif robot == 'real':
        name, args = 'real_bringup.launch.py', {'rviz': 'false', 'agent': 'true'}
    else:
        raise RuntimeError(f'robot 은 sim 또는 real 이어야 합니다 (받은 값: {robot})')
    return [GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', name)), launch_arguments=args.items())])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', description='sim 또는 real'),
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo 화면 (시뮬레이션만)'),
        OpaqueFunction(function=setup),
    ])
