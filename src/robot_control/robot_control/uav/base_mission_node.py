#!/usr/bin/env python3
"""
미션 노드의 공통 기반 클래스
PX4 오프보드 제어, TF 관리, 상태 머신 골격 등의 공통 기능을 제공합니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from abc import ABC, abstractmethod

# ROS2 메시지 임포트
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

# TF2 관련 모듈
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# 로컬 유틸리티 모듈
from ..utils import drone_control_utils as dcu


class BaseMissionNode(Node, ABC):
    """
    미션 노드의 공통 기반 클래스.
    
    이 클래스는 PX4 드론을 제어하는 미션 노드들의 공통 기능을 제공합니다:
    - ROS2 퍼블리셔/서브스크라이버 설정
    - TF 변환 관리
    - 공통 상태 변수 관리
    - 기본 상태 머신 골격
    - 공통 콜백 함수들
    
    자식 클래스는 run_mission_logic() 메서드를 구현하여 고유한 미션 로직을 정의해야 합니다.
    """
    
    def __init__(self, node_name: str, drone_frame_id: str = "x500_gimbal_0"):
        super().__init__(node_name)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        
        # --- QoS 프로파일 설정 ---
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.visual_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=30
        )
        
        # --- 퍼블리셔 설정 ---
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.state_publisher = self.create_publisher(
            String, "/drone/state", 10
        )
        self.visual_marker_publisher = self.create_publisher(
            MarkerArray, "/mission_visuals", self.visual_qos_profile
        )
        
        # --- 서브스크라이버 설정 ---
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", 
            self.local_position_callback, self.qos_profile
        )
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude",
            self.attitude_callback, self.qos_profile
        )
        
        # --- TF 설정 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- 공통 상태 변수 ---
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.current_attitude = None
        self.drone_frame_id = drone_frame_id
        
        # --- 핸드셰이크 관련 변수 ---
        self.handshake_counter = 0
        self.handshake_duration = 15
        
        # --- 상태 머신 타이머 (10Hz) ---
        self.state_machine_timer = self.create_timer(0.1, self.run_state_machine_wrapper)
        
        self.get_logger().info(f"{node_name} initialized successfully.")
    
    # --- 공통 콜백 함수들 ---
    
    def local_position_callback(self, msg: VehicleLocalPosition):
        """Local position 메시지 콜백"""
        self.current_local_pos = msg
    
    def attitude_callback(self, msg: VehicleAttitude):
        """Attitude 메시지 콜백"""
        self.current_attitude = msg
    
    # --- TF 관련 메서드 ---
    
    def update_current_map_pose(self):
        """
        드론의 현재 map 좌표계 위치를 TF로부터 업데이트합니다.
        
        Returns:
            bool: TF 조회 성공 여부
        """
        try:
            # TF lookup용 완전한 프레임 ID 구성 (base_link 접미사 추가)
            full_drone_frame_id = f"{self.drone_frame_id}/base_link"
            trans = self.tf_buffer.lookup_transform('map', full_drone_frame_id, rclpy.time.Time())
            
            if self.current_map_pose is None:
                self.current_map_pose = PoseStamped()
            
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            self.current_map_pose.pose.orientation = trans.transform.rotation
            
            return True
            
        except TransformException as e:
            if self.state != "INIT":
                self.get_logger().warn(
                    f"TF lookup failed for '{full_drone_frame_id}': {e}", 
                    throttle_duration_sec=1.0
                )
            return False
    
    # --- 상태 머신 관련 메서드 ---
    
    def run_state_machine_wrapper(self):
        """
        상태 머신 실행 래퍼 함수.
        필수 데이터 확인 후 미션별 로직을 호출합니다.
        """
        # 필수 데이터 확인
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return
        
        # 상태 퍼블리시
        self.state_publisher.publish(String(data=self.state))
        
        # Offboard 제어 모드 퍼블리시 (특정 상태 제외)
        if self.state not in ["LANDING", "LANDED", "INIT"]:
            dcu.publish_offboard_control_mode(self)
        
        # 공통 상태 처리
        self._handle_common_states()
        
        # 미션별 로직 실행
        self.run_mission_logic()
    
    def _handle_common_states(self):
        """공통 상태들을 처리합니다."""
        
        if self.state == "INIT":
            self.get_logger().info("✅ 시스템 준비 완료. 미션 시작을 기다리는 중...", once=True)
            
        elif self.state == "HANDSHAKE":
            # 현재 위치 유지하면서 핸드셰이크
            if self.current_local_pos:
                sp_msg = TrajectorySetpoint(
                    position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z],
                    timestamp=int(self.get_clock().now().nanoseconds / 1000)
                )
                self.trajectory_setpoint_publisher.publish(sp_msg)
            
            # ARM 및 Offboard 모드 설정
            dcu.arm_and_offboard(self)
            
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("🔧 핸드셰이크 완료. ARM 및 Offboard 모드 활성화.")
                self.state = "ARMED_IDLE"
                
        elif self.state == "ARMED_IDLE":
            # 현재 위치에서 대기
            if self.current_local_pos:
                sp_msg = TrajectorySetpoint(
                    position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z],
                    timestamp=int(self.get_clock().now().nanoseconds / 1000)
                )
                self.trajectory_setpoint_publisher.publish(sp_msg)
                
        elif self.state == "LANDING":
            self.get_logger().info("🛬 착륙 중...", throttle_duration_sec=5.0)
            dcu.land_drone(self)
            # 착륙 감지는 자식 클래스에서 구현
            
        elif self.state == "LANDED":
            self.get_logger().info("✅ 착륙 완료.", once=True)
    
    # --- 유틸리티 메서드들 ---
    
    def start_mission(self):
        """미션을 시작합니다 (INIT → HANDSHAKE)."""
        if self.state == "INIT":
            self.get_logger().info("🚁 미션 시작!")
            self.state = "HANDSHAKE"
        else:
            self.get_logger().warn(f"미션을 시작할 수 없습니다. 현재 상태: {self.state}")
    
    def emergency_land(self):
        """비상 착륙을 실행합니다."""
        if self.state not in ["LANDING", "LANDED"]:
            self.get_logger().warn("⚠️ 비상 착륙 실행!")
            self.state = "LANDING"
    
    def check_arrival(self, target_pos, tolerance=2.0):
        """
        목표 지점 도착 여부를 확인합니다.
        
        Args:
            target_pos: 목표 위치 [x, y, z] 또는 PoseStamped
            tolerance: 허용 오차 (미터)
            
        Returns:
            bool: 도착 여부
        """
        return dcu.check_arrival(self.current_map_pose, target_pos, tolerance)
    
    def point_gimbal_at_target(self, target_enu_pos):
        """
        짐벌을 특정 ENU 좌표로 향하게 합니다.
        
        Args:
            target_enu_pos: 목표 ENU 좌표 [x, y, z]
        """
        dcu.point_gimbal_at_target(self, self.current_map_pose, target_enu_pos)
    
    def publish_position_setpoint(self, target_map_pos):
        """
        Map 좌표계 기준 위치 세트포인트를 퍼블리시합니다.
        
        Args:
            target_map_pos: 목표 map 좌표 [x, y, z]
        """
        dcu.publish_position_setpoint(
            self, self.current_local_pos, self.current_map_pose, target_map_pos
        )
    
    # --- 추상 메서드 ---
    
    @abstractmethod
    def run_mission_logic(self):
        """
        미션별 고유 로직을 구현하는 추상 메서드.
        
        자식 클래스에서 반드시 구현해야 하며, 이 함수에서는:
        1. 미션별 상태 처리 (예: MOVING, HOVERING)
        2. 사용자 입력 처리 (대화형 미션의 경우)
        3. 웨이포인트 순회 로직 (자동 미션의 경우)
        4. 시각화 마커 퍼블리시
        등을 처리해야 합니다.
        """
        pass
    
    # --- 선택적 오버라이드 메서드 ---
    
    def on_mission_complete(self):
        """
        미션 완료 시 호출되는 메서드.
        자식 클래스에서 필요시 오버라이드할 수 있습니다.
        """
        self.get_logger().info("🏁 미션 완료!")
    
    def on_emergency_stop(self):
        """
        비상 정지 시 호출되는 메서드.
        자식 클래스에서 필요시 오버라이드할 수 있습니다.
        """
        self.get_logger().warn("🚨 비상 정지!")
        self.emergency_land()
    
    # --- 소멸자 ---
    
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        if hasattr(self, 'state_machine_timer'):
            self.state_machine_timer.cancel()
        super().destroy_node() 