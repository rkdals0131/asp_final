from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # =================================================================
        # Original: topic_bridge.launch.py
        # =================================================================

        # TF, clock, odometry, cmd_vel bridge
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                # Clock
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # TF from Gazebo
                '/model/X1_asp/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/X1_asp/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/x500_gimbal_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/x500_gimbal_0/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Command & Odometry
                '/model/X1_asp/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/X1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        ),

        # X1 Camera bridge (image + camera_info)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/X1_asp/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/default/model/X1_asp/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # X1 LiDAR bridge (point cloud)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/X1_asp/link/base_link/sensor/gpu_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),

        # x500 Camera bridge (image + camera_info)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # =================================================================
        # Original: pose_tf_broadcaster.launch.py
        # =================================================================
        
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
                '0.43', '0.0', '0.26',  # translation (x, y, z)
                '0', '0', '0',          # rotation (rpy in radians)
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

        # =================================================================
        # NEW: MicroXRCEAgent
        # =================================================================
        
        # MicroXRCEAgent 실행
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        )
    ])