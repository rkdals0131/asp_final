#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 디렉토리 찾기
    pkg_share = FindPackageShare('robot_control')
    
    # --- 파일 경로 구성 ---
    ugv_wp_csv = PathJoinSubstitution([
        pkg_share, 'config', 'ugv_wp.csv'
    ])
    ugv_params = PathJoinSubstitution([
        pkg_share, 'config', 'ugv_params.yaml'
    ])

    
    # --- 노드 정의 ---
    
    # 1. Path Follower Node
    path_follower_node = Node(
        package='robot_control',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            ugv_params,
            {'waypoint_file': ugv_wp_csv},
        ]
    )
    
    return LaunchDescription([
        path_follower_node,
    ])
