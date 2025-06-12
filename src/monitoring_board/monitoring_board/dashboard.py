#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String

# --- TF2 관련 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import os
import datetime

class UAVDashboard(Node):
    """
    드론과 차량의 주요 정보를 터미널에 실시간으로 표시하는 대시보드 노드.
    RViz나 여러 topic echo 대신, 핵심 정보를 한눈에 파악하기 위해 사용됩니다.
    """

    def __init__(self):
        super().__init__('uav_dashboard_node')
        # 시뮬레이션 시간을 사용하도록 설정
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # --- 데이터 저장을 위한 멤버 변수 초기화 ---
        self.drone_local_pos = None     # 드론 로컬 위치 (PX4 기준)
        self.vehicle_odom = None        # 차량 오도메트리 (속도 포함)
        self.drone_world_pos = None     # 드론 월드 위치 (map 기준)
        self.vehicle_world_pos = None   # 차량 월드 위치 (map 기준)
        
        # 상태 정보 (현재는 플레이스홀더)
        self.drone_state = "UNKNOWN"
        self.vehicle_state = "UNKNOWN"

        # TF 프레임 이름
        self.drone_frame_id = "x500_gimbal_0/base_link"
        self.vehicle_frame_id = "X1_asp/base_link"

        # --- TF2 Listener 초기화 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber 초기화 ---
        # 토픽별로 QoS 설정이 다르므로, 각각에 맞는 프로파일을 생성합니다.

        # PX4의 vehicle_local_position 토픽은 Best Effort, Volatile로 전송됩니다.
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )

        # Gazebo의 odometry 토픽과 상태(state) 토픽은 Reliable, Volatile로 전송됩니다.
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        # 드론 로컬 위치 구독 (Best Effort)
        self.create_subscription(
            VehicleLocalPosition, 
            "/fmu/out/vehicle_local_position", 
            self.drone_local_pos_callback, 
            qos_profile_best_effort
        )
        # 차량 오도메트리 구독 (Reliable)
        self.create_subscription(
            Odometry, 
            "/model/X1/odometry", 
            self.vehicle_odometry_callback, 
            qos_profile_reliable
        )

        # 드론 상태 구독 (Reliable)
        self.create_subscription(
            String, "/drone/state", self.drone_state_callback, qos_profile_reliable
        )

        # 차량 상태 구독 (Reliable)
        self.create_subscription(
            String, "/vehicle/state", self.vehicle_state_callback, qos_profile_reliable
        )

        # --- 30Hz 업데이트를 위한 타이머 생성 ---
        self.timer = self.create_timer(1.0 / 30.0, self.update_dashboard)
        
        self.get_logger().info("UAV Telemetry Dashboard is running.")

    def drone_local_pos_callback(self, msg: VehicleLocalPosition):
        self.drone_local_pos = msg

    def drone_state_callback(self, msg: String):
        self.drone_state = msg.data

    def vehicle_state_callback(self, msg: String):
        self.vehicle_state = msg.data

    def vehicle_odometry_callback(self, msg: Odometry):
        self.vehicle_odom = msg

    def update_tf_poses(self):
        """TF Listener를 사용하여 드론과 차량의 월드 좌표를 업데이트합니다."""
        try:
            # 드론 월드 좌표 조회
            trans_drone = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            self.drone_world_pos = trans_drone.transform.translation
        except TransformException:
            self.drone_world_pos = None

        try:
            # 차량 월드 좌표 조회
            trans_vehicle = self.tf_buffer.lookup_transform('map', self.vehicle_frame_id, rclpy.time.Time())
            self.vehicle_world_pos = trans_vehicle.transform.translation
        except TransformException:
            self.vehicle_world_pos = None

    def format_point(self, point: Point, default_text="Waiting..."):
        """Point 메시지를 포맷된 문자열로 변환합니다."""
        if point:
            return f"X: {point.x:6.2f} | Y: {point.y:6.2f} | Z: {point.z:6.2f}"
        return default_text

    def format_velocity(self, velocity, default_text="Waiting..."):
        """Velocity 메시지를 포맷된 문자열로 변환합니다."""
        if velocity:
            return f"X: {velocity.x:6.2f} | Y: {velocity.y:6.2f} | Z: {velocity.z:6.2f}"
        return default_text

    def update_dashboard(self):
        """화면을 지우고 최신 정보로 대시보드를 다시 그립니다."""
        # 1. TF 정보 업데이트
        self.update_tf_poses()

        # 2. 화면 클리어
        os.system('clear')

        # 3. 데이터 포매팅
        drone_world_str = self.format_point(self.drone_world_pos)
        drone_local_str = "Waiting..."
        if self.drone_local_pos:
            # VehicleLocalPosition은 Point 타입이 아니므로 직접 포맷
            drone_local_str = f"X: {self.drone_local_pos.x:6.2f} | Y: {self.drone_local_pos.y:6.2f} | Z: {self.drone_local_pos.z:6.2f}"

        vehicle_world_str = self.format_point(self.vehicle_world_pos)
        vehicle_odom_pos_str = self.format_point(self.vehicle_odom.pose.pose.position if self.vehicle_odom else None)
        vehicle_vel_str = self.format_velocity(self.vehicle_odom.twist.twist.linear if self.vehicle_odom else None)

        # 4. 대시보드 문자열 생성
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        dashboard = f"""
======================================================================
                  UAV & VEHICLE TELEMETRY DASHBOARD
======================================================================
-- DRONE (x500_gimbal_0) ---------------------------------------------
  [STATE]         : {self.drone_state}
  [WORLD POS (m)] : {drone_world_str}
  [LOCAL POS (m)] : {drone_local_str}

-- VEHICLE (X1_asp) --------------------------------------------------
  [STATE]         : {self.vehicle_state}
  [WORLD POS (m)] : {vehicle_world_str}
  [ODOM POS (m)]  : {vehicle_odom_pos_str}
  [VELOCITY(m/s)] : {vehicle_vel_str}
----------------------------------------------------------------------
  Last updated: {timestamp}
"""

        # 5. 대시보드 출력
        print(dashboard)


def main(args=None):
    rclpy.init(args=args)
    dashboard_node = UAVDashboard()
    rclpy.spin(dashboard_node)
    dashboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
