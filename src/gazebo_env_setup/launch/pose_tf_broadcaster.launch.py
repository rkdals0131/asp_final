from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # TF 브로드캐스터 노드
        Node(
            package='gazebo_env_setup',
            executable='pose_tf_broadcaster',
            name='pose_tf_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Static TF: base_link → camera_front
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.43', '0.0', '0.26',  # translation
                '0', '0', '0',          # rotation (rpy in degrees)
                'X1_asp/base_link', 'X1_asp/base_link/camera_front'
            ],
            output='screen'
        ),

        # Static TF: base_link → gpu_lidar
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0.60', '0.0', '0.13',
                '0', '0', '0',
                'X1_asp/base_link', 'X1_asp/base_link/gpu_lidar'
            ],
            output='screen'
        ),
    ])
