#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 디렉토리 찾기
    pkg_share = FindPackageShare('robot_control')
    
    # 파일 경로 구성
    csv_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'ugv_wp.csv'
    ])
    
    params_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        'ugv_params.yaml'
    ])
    
    # Path Follower Node
    path_follower_node = Node(
        package='robot_control',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file_path,
            {
                'waypoint_file': csv_file_path,
            }
        ]
    )
    
    return LaunchDescription([
        path_follower_node,
    ])
