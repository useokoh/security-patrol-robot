"""5~6단계: 이상 감지, 사건 관리, 텔레그램 알림 노드 실행 (nav.launch.py 가 켜져 있는 상태에서 실행).

  ros2 launch security_patrol detect.launch.py
  # 8단계 실제 로봇 (real_nav.launch.py 가 켜져 있는 상태):
  ros2 launch security_patrol detect.launch.py use_sim_time:=false route_file:=$HOME/maps_real/patrol_route_real.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('security_patrol')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'anomaly_detector.yaml'),
            description='이상 감지 설정 파일'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='시뮬레이션 시계 사용 (실제 로봇은 false)'),
        DeclareLaunchArgument(
            'route_file', default_value=os.path.join(pkg_share, 'config', 'patrol_route.yaml'),
            description="알림의 '어느 지점 근처' 에 쓸 순찰 경로 파일"),

        # 설정 파일 뒤에 넣은 값이 파일의 use_sim_time 을 덮어쓴다
        Node(
            package='security_patrol', executable='anomaly_detector', name='anomaly_detector',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}]),

        Node(
            package='security_patrol', executable='incident_manager', name='incident_manager',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}]),

        Node(
            package='security_patrol', executable='alert_notifier', name='alert_notifier',
            output='screen', parameters=[{'route_file': LaunchConfiguration('route_file')}]),
    ])
