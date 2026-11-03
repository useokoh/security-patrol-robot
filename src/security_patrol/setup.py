import os
from glob import glob

from setuptools import setup

package_name = 'security_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dony',
    maintainer_email='277463901+useokoh@users.noreply.github.com',
    description='출입 통제 구역 경비 순찰 로봇',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_check = security_patrol.scan_check:main',
            'motion_test = security_patrol.motion_test:main',
            'auto_mapper = security_patrol.auto_mapper:main',
            'nav_test = security_patrol.nav_test:main',
            'loc_monitor = security_patrol.loc_monitor:main',
            'patrol_node = security_patrol.patrol_node:main',
            'anomaly_detector = security_patrol.anomaly_detector:main',
            'incident_manager = security_patrol.incident_manager:main',
            'intruder_walker = security_patrol.intruder_walker:main',
            'alert_notifier = security_patrol.alert_notifier:main',
            'telegram_setup = security_patrol.telegram_setup:main',
            'eval_run = security_patrol.eval_run:main',
            'robot_bridge = security_patrol.robot_bridge:main',
            'route_recorder = security_patrol.route_recorder:main',
            'wheel_check = security_patrol.wheel_check:main',
        ],
    },
)
