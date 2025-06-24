from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.admin',
        f'{package_name}.uav',
        f'{package_name}.ugv',
        f'{package_name}.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 파일들 추가
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
            
        # Config 파일들 추가  
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.csv'))),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='user1',
    maintainer_email='kikiws70@gmail.com',
    description='통합 UAV 및 UGV 제어 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = robot_control.admin.dashboard_node:main',
            'mission_control_node = robot_control.admin.mission_control_node:main',
            'interactive_mission = robot_control.uav.interactive_mission:main',
            'waypoint_mission = robot_control.uav.waypoint_mission:main',
            'path_follower_node = robot_control.ugv.path_follower_node:main',
            'path_planner = robot_control.ugv.path_planner:main',
            'detected_marker_visualizer = robot_control.uav.detected_marker_visualizer:main',
        ],
    },
)