from setuptools import find_packages, setup

package_name = 'uav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'px4_msgs'],
    zip_safe=True,
    maintainer='user1',
    maintainer_email='kikiws70@gmail.com',
    description='UAV Controller package for PX4 offboard control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_mission = uav_controller.interactive_mission:main',
            'waypoint_mission = uav_controller.waypoint_mission:main',
        ],
    },
)
