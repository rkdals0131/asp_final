#!/usr/bin/env python3
"""
미션 노드의 공통 기반 클래스
PX4 오프보드 제어, TF 관리, 상태 머신 골격 등의 공통 기능을 제공
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from abc import ABC, abstractmethod
from rclpy.duration import Duration

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
    미션 노드의 공통 기반 클래스
    
    이 클래스는 PX4 드론을 제어하는 미션 노드들의 공통 기능을 제공:
    - ROS2 퍼블리셔/서브스크라이버 설정
    - TF 변환 관리
    - 공통 상태 변수 관리
    - 기본 상태 머신 골격
    - 공통 콜백 함수들
    - 공통 미션 데이터 (웨이포인트, 주시 타겟)
    
    자식 클래스는 run_mission_logic() 메서드를 구현하여 고유한 미션 로직을 정의해야 함
    """
    
    def __init__(self, node_name: str, drone_frame_id: str = "x500_gimbal_0"):
        super().__init__(node_name)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        
        # QoS 프로파일 설정
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
        
        # 퍼블리셔 설정
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
        
        # 서브스크라이버 설정
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", 
            self.local_position_callback, self.qos_profile
        )
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude",
            self.attitude_callback, self.qos_profile
        )
        
        # TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 공통 상태 변수
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.current_attitude = None
        self.drone_frame_id = drone_frame_id
        self.vehicle_frame_id = "X1_asp"  # UGV 프레임 ID 추가
        
        # 착륙 감지 관련 변수
        self.landed_check_start_time = None
        self.landed_check_duration = Duration(seconds=2.0)
        
        # 핸드셰이크 관련 변수
        self.handshake_counter = 0
        self.handshake_duration = 15
        
        # 공통 미션 데이터
        self._setup_mission_data()
        
        # 상태 머신 타이머 (10Hz)
        self.state_machine_timer = self.create_timer(0.1, self.run_state_machine_wrapper)
        
        self.get_logger().info(f"{node_name} initialized successfully.")
    
    def _setup_mission_data(self):
        """공통 미션 데이터를 설정"""
        # 미션 정의: (x, y, z, yaw, stare_index)
        # yaw는 맵 좌표계 기준 (X축이 0도, 반시계방향이 양수)
        self.mission_definition = [
            (-95, 80, 17, 315, 0),  #wp 0
            (-80, 80, 33, 315, 1),   #wp 1
            (-63, 75, 25, 180, 2),   #wp 2
            (-55, 72, 15, 180, 3),   #wp 3
            (-70, 100, 17, 160, 4),  #wp 4
            (-90, 100, 17, 170, 5),  #wp 5
            (-93, 96, 22, 170, 6),   #wp 6
            (-100, 95, 30, 170, 7),   #wp 7
            (-63, 100, 10, 270, 8),  #wp 8
        ]
        
        # 드론 웨이포인트 (x, y, z 좌표만 추출)
        self.drone_waypoints = np.array([p[0:3] for p in self.mission_definition], dtype=np.float64)
        
        # 각 웨이포인트에서의 목표 yaw 각도 (맵 좌표계 기준)
        self.waypoint_yaws = np.array([p[3] for p in self.mission_definition], dtype=np.float64)
        
        # 각 웨이포인트에서 응시할 주시 타겟 인덱스
        self.stare_indices = np.array([p[4] for p in self.mission_definition])
        
        # 주시 타겟들 (짐벌이 바라볼 목표 좌표들)
        self.stare_targets = [
            [-94.4088, 68.4708, 3.8531],    #0
            [-75.4421, 74.9961, 23.2347],   #1
            [-75.0, 75.0, 20.0],             #2
            [-75.0, 75.0, 10.0],             #3
            [-82.7931, 113.4203, 3.8079],   #4
            [-97.9238, 105.2799, 8.5504],   #5
            [-109.0, 100.0, 19.0],           #6
            [-109.1330, 100.3533, 23.1363], #7
            [-62.9630, 99.0915, 0.1349]     #8
        ]
        
        # 최종 목적지 (편의를 위한 별칭)
        self.final_destination = self.stare_targets[-1]
    
    # 공통 콜백 함수들
    
    def local_position_callback(self, msg: VehicleLocalPosition):
        """Local position 메시지 콜백"""
        self.current_local_pos = msg
    
    def attitude_callback(self, msg: VehicleAttitude):
        """Attitude 메시지 콜백"""
        self.current_attitude = msg
    
    # TF 관련 메서드
    
    def update_current_map_pose(self):
        """
        드론의 현재 map 좌표계 위치를 TF로부터 업데이트
        
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
    
    # 상태 머신 관련 메서드
    
    def run_state_machine_wrapper(self):
        """
        상태 머신 실행 래퍼 함수
        필수 데이터 확인 후 미션별 로직을 호출
        """
        # 필수 데이터 확인
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return
        
        # 상태 퍼블리시
        self.state_publisher.publish(String(data=self.state))
        
        # Offboard 제어 모드 퍼블리시 (특정 상태 제외)
        if self.state not in ["LANDING", "LANDED", "INIT", "DISARMED"]:
            dcu.publish_offboard_control_mode(self)
        
        # 공통 상태 처리
        self._handle_common_states()
        
        # 미션별 로직 실행
        self.run_mission_logic()
    
    def _handle_common_states(self):
        """공통 상태들을 처리"""
        
        if self.state == "INIT":
            self.get_logger().info("시스템 준비 완료. 미션 시작을 기다리는 중...", once=True)
            
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
                self.get_logger().info("핸드셰이크 완료. ARM 및 Offboard 모드 활성화")
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
            self.get_logger().info("착륙 중...", throttle_duration_sec=5.0)
            dcu.land_drone(self)
            self.check_landed_on_vehicle() # UGV 위 착륙 감지 로직 호출
            
        elif self.state == "LANDED":
            self.get_logger().info("착륙 완료. Disarm 실행")
            dcu.disarm_drone(self)
            self.state = "DISARMED"
            
        elif self.state == "DISARMED":
            self.get_logger().info("시동 꺼짐. 미션 종료", once=True)
            # 최종 상태, 아무것도 하지 않음
            pass
    
    # 유틸리티 메서드들
    
    def check_landed_on_vehicle(self, xy_tolerance=0.5, z_tolerance=0.6, vel_tolerance=0.2):
        """UGV 위에 착륙했는지 상대 거리와 속도를 기준으로 확인"""
        try:
            # UGV의 TF 조회
            vehicle_trans = self.tf_buffer.lookup_transform(
                'map', f"{self.vehicle_frame_id}/base_link", rclpy.time.Time()
            )
            
            # 드론과 UGV 간의 거리 계산
            dx = self.current_map_pose.pose.position.x - vehicle_trans.transform.translation.x
            dy = self.current_map_pose.pose.position.y - vehicle_trans.transform.translation.y
            dz = self.current_map_pose.pose.position.z - vehicle_trans.transform.translation.z
            dist_xy = np.sqrt(dx**2 + dy**2)
            dist_z = abs(dz)

            # 수직 속도 확인
            vz = self.current_local_pos.vz if self.current_local_pos else 1.0

            # 디버깅 로그 추가
            self.get_logger().info(f"[LANDING_CHECK] dist_xy: {dist_xy:.3f}m, dist_z: {dist_z:.3f}m, vz: {vz:.3f}m/s", throttle_duration_sec=1.0)

            # 착륙 조건 확인
            if dist_xy < xy_tolerance and dist_z < z_tolerance and abs(vz) < vel_tolerance:
                now = self.get_clock().now()
                if self.landed_check_start_time is None:
                    self.landed_check_start_time = now
                    self.get_logger().info("착륙 가능성 감지. 안정화 상태 확인 시작...")
                
                # 일정 시간 동안 조건이 유지되었는지 확인
                if now - self.landed_check_start_time > self.landed_check_duration:
                    self.get_logger().info("UGV 위 착륙 확인! 상태를 LANDED로 변경")
                    self.state = "LANDED"
            else:
                # 조건이 깨지면 타이머 리셋
                if self.landed_check_start_time is not None:
                    self.get_logger().info("착륙 조건 벗어남. 타이머 리셋")
                self.landed_check_start_time = None

        except TransformException as e:
            self.get_logger().warn(
                f"UGV TF({self.vehicle_frame_id}) 조회 실패: {e}",
                throttle_duration_sec=2.0
            )
            self.landed_check_start_time = None

    def start_mission(self):
        """미션을 시작 (INIT → HANDSHAKE)"""
        if self.state == "INIT":
            self.get_logger().info("미션 시작!")
            self.state = "HANDSHAKE"
        else:
            self.get_logger().warn(f"미션을 시작할 수 없음. 현재 상태: {self.state}")
    
    def emergency_land(self):
        """비상 착륙을 실행"""
        if self.state not in ["LANDING", "LANDED"]:
            self.get_logger().warn("비상 착륙 실행!")
            self.state = "LANDING"
    
    def check_arrival(self, target_pos, tolerance=2.0):
        """
        목표 지점 도착 여부를 확인
        
        Args:
            target_pos: 목표 위치 [x, y, z] 또는 PoseStamped
            tolerance: 허용 오차 (미터)
            
        Returns:
            bool: 도착 여부
        """
        return dcu.check_arrival(self.current_map_pose, target_pos, tolerance)
    
    def point_gimbal_at_target(self, target_enu_pos):
        """
        짐벌을 특정 ENU 좌표로 향하게 함
        
        Args:
            target_enu_pos: 목표 ENU 좌표 [x, y, z]
        """
        dcu.point_gimbal_at_target(self, self.current_map_pose, target_enu_pos)
    
    def publish_position_setpoint(self, target_map_pos, target_yaw_deg=None):
        """
        Map 좌표계 기준 위치 세트포인트를 퍼블리시
        
        Args:
            target_map_pos: 목표 map 좌표 [x, y, z]
            target_yaw_deg: 목표 yaw 각도 (도 단위, 맵 좌표계 기준, None이면 yaw 제어 안함)
        """
        dcu.publish_position_setpoint(
            self, self.current_local_pos, self.current_map_pose, target_map_pos, target_yaw_deg
        )
    
    def publish_waypoint_setpoint(self, waypoint_index):
        """
        웨이포인트 인덱스를 기반으로 위치 및 yaw 세트포인트를 퍼블리시
        
        Args:
            waypoint_index: 웨이포인트 인덱스 (0부터 시작)
        """
        if 0 <= waypoint_index < len(self.drone_waypoints):
            target_pos = self.drone_waypoints[waypoint_index].tolist()
            target_yaw = self.waypoint_yaws[waypoint_index]
            self.publish_position_setpoint(target_pos, target_yaw)
        else:
            self.get_logger().error(f"잘못된 웨이포인트 인덱스: {waypoint_index}")
    
    def get_waypoint_position(self, waypoint_index):
        """
        웨이포인트의 위치를 반환
        
        Args:
            waypoint_index: 웨이포인트 인덱스
            
        Returns:
            list: [x, y, z] 좌표 또는 None (잘못된 인덱스)
        """
        if 0 <= waypoint_index < len(self.drone_waypoints):
            return self.drone_waypoints[waypoint_index].tolist()
        return None
    
    def get_waypoint_yaw(self, waypoint_index):
        """
        웨이포인트의 yaw 각도를 반환
        
        Args:
            waypoint_index: 웨이포인트 인덱스
            
        Returns:
            float: yaw 각도 (도 단위, 맵 좌표계 기준) 또는 None
        """
        if 0 <= waypoint_index < len(self.waypoint_yaws):
            return float(self.waypoint_yaws[waypoint_index])
        return None
    
    # 추상 메서드
    
    @abstractmethod
    def run_mission_logic(self):
        """
        미션별 고유 로직을 구현하는 추상 메서드
        
        자식 클래스에서 반드시 구현해야 하며, 이 함수에서는:
        1. 미션별 상태 처리 (예: MOVING, HOVERING)
        2. 사용자 입력 처리 (대화형 미션의 경우)
        3. 웨이포인트 순회 로직 (자동 미션의 경우)
        4. 시각화 마커 퍼블리시
        등을 처리해야 함
        """
        pass
    
    # 선택적 오버라이드 메서드
    
    def on_mission_complete(self):
        """
        미션 완료 시 호출되는 메서드
        자식 클래스에서 필요시 오버라이드할 수 있음
        """
        self.get_logger().info("미션 완료!")
    
    def on_emergency_stop(self):
        """
        비상 정지 시 호출되는 메서드
        자식 클래스에서 필요시 오버라이드할 수 있음
        """
        self.get_logger().warn("비상 정지!")
        self.emergency_land()
    
    # 소멸자
    
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        if hasattr(self, 'state_machine_timer'):
            self.state_machine_timer.cancel()
        super().destroy_node() 