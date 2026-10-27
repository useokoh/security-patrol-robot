"""출입 통제 구역 Gazebo(Fortress) 월드를 띄우고 순찰 로봇(ESP32 로봇 모델)을 스폰한다.

구성:
  Gazebo Fortress  ──(ros_gz_bridge)──  ROS2 토픽 /cmd_vel /odom /tf /joint_states /scan /clock
  robot_state_publisher : URDF → 로봇 내부 TF (base_link → laser_frame 등)
  rviz2                 : 라이다/로봇 시각화

사용 예:
  ros2 launch security_patrol sim_world.launch.py
  ros2 launch security_patrol sim_world.launch.py gui:=false rviz:=false
  ros2 launch security_patrol sim_world.launch.py render_engine:=ogre   # 화면이 깨질 때
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROBOT_NAME = 'patrol_bot'
WORLD_NAME = 'restricted_area'  # worlds/restricted_area.world 의 <world name> 과 같아야 한다


def gazebo(context):
    """launch 인자를 조합해 `ign gazebo` 실행 옵션을 만든다."""
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    engine = LaunchConfiguration('render_engine').perform(context)

    # -r: 바로 시뮬레이션 시작, -s: 화면 없이 서버만
    gz_args = f'-r {"" if gui else "-s "}--render-engine {engine} {world}'
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': gz_args}.items())]


def generate_launch_description():
    pkg_share = get_package_share_directory('security_patrol')
    urdf = os.path.join(pkg_share, 'urdf', 'patrol_bot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' use_sim:=true']), value_type=str)

    # 형식: 토픽@ROS타입[Gazebo타입  ( [ : Gazebo→ROS,  ] : ROS→Gazebo )
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'/model/{ROBOT_NAME}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/model/{ROBOT_NAME}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'/world/{WORLD_NAME}/model/{ROBOT_NAME}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds', 'restricted_area.world'),
            description='Gazebo 월드 파일 경로'),
        DeclareLaunchArgument('x_pose', default_value='-3.0', description='로봇 시작 x [m]'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='로봇 시작 y [m]'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 화면 표시 여부'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2',
            description='Gazebo 렌더링 엔진 (가상머신에서 화면/라이다가 깨지면 ogre)'),

        OpaqueFunction(function=gazebo),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-topic', 'robot_description', '-name', ROBOT_NAME,
                       '-x', LaunchConfiguration('x_pose'),
                       '-y', LaunchConfiguration('y_pose'), '-z', '0.02']),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            output='screen', arguments=bridge_topics,
            parameters=[{'use_sim_time': True}],
            # Gazebo 쪽 긴 토픽 이름을 실제 로봇과 같은 ROS 토픽 이름으로 바꾼다
            remappings=[
                (f'/model/{ROBOT_NAME}/odometry', '/odom'),
                (f'/model/{ROBOT_NAME}/pose', '/tf'),
                (f'/world/{WORLD_NAME}/model/{ROBOT_NAME}/joint_state', '/joint_states'),
            ]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'sim_view.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
