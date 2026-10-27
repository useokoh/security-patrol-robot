"""5단계: 이상 감지 노드 실행 (nav.launch.py 가 켜져 있는 상태에서 실행).

  ros2 launch security_patrol detect.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('security_patrol')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'anomaly_detector.yaml'),
            description='이상 감지 설정 파일'),

        Node(
            package='security_patrol', executable='anomaly_detector', name='anomaly_detector',
            output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
