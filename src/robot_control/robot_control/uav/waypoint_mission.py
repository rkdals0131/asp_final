#!/usr/bin/env python3
"""
웨이포인트 자동 순회 및 정밀 착륙 미션 노드
지정된 웨이포인트를 순서대로 방문하고, 마지막 지점에서 아루코 마커를 이용해 정밀 착륙을 수행
"""

import rclpy
import threading
import sys
import math
import copy

from std_msgs.msg import String, Header
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import VehicleLandDetected, VehicleControlMode, VehicleStatus  # 착륙 감지 및 제어 상태 메시지
from mission_admin_interfaces.srv import MissionComplete
from rcl_interfaces.msg import ParameterDescriptor

from .base_mission_node import BaseMissionNode
from ..utils import drone_control_utils as dcu
from ..utils import viz_factory as visu


class WaypointMissionNode(BaseMissionNode):
    """
    웨이포인트 기반 자동 미션 및 정밀 착륙을 수행하는 노드.
    지정된 웨이포인트로 이동하며, 각 지점에서 Stare 타겟을 응시하고 2초간 호버링
    마지막 웨이포인트에서는 ArUco 마커를 이용한 정밀 착륙을 수행
    """
    
    def __init__(self):
        super().__init__('waypoint_mission_node')
        
        # 추가 서브스크라이버 (미션 컨트롤 연동)
        self.mission_command_sub = self.create_subscription(
            String, "/drone/mission_command", self.mission_command_callback, 10
        )
        
        # PX4 상태 구독자
        self.land_detector_sub = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._land_detected_callback, self.qos_profile
        )
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, "/fmu/out/vehicle_control_mode", self._vehicle_control_mode_callback, self.qos_profile
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._vehicle_status_callback, self.qos_profile
        )
        
        # 미션 컨트롤 서비스 클라이언트
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')
        
        # 웨이포인트 미션 관련 변수
        self.current_waypoint_index = 0
        
        # 짐벌 카메라 프레임 ID 파라미터로 로드 (config에서 받아옴)
        self.declare_parameter('gimbal_camera_frame', 'x500_gimbal_0/camera_link')
        self.gimbal_camera_frame_id = self.get_parameter('gimbal_camera_frame').value
        
        # 정밀 착륙 관련 변수 및 파라미터
        self.declare_parameter('landing_altitude', 0.5)
        self.declare_parameter('descent_speed', 1.0, 
            ParameterDescriptor(description="마커 정렬 후 최종 착륙 시 하강 속도 (m/s)"))
        self.declare_parameter('horizontal_tolerance', 0.15)
        self.declare_parameter('vertical_tolerance', 0.3)
        self.declare_parameter('landing_marker_id', 10)  # 착륙용 마커 ID
        self.declare_parameter('search_descent_speed', 1.0, 
            ParameterDescriptor(description="마커를 탐색하며 하강할 때의 속도 (m/s)"))
        self.declare_parameter('precision_horizontal_tolerance', 0.1)  # 정밀 착륙 시 수평 허용 오차
        
        self.landing_altitude = self.get_parameter('landing_altitude').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.horizontal_tolerance = self.get_parameter('horizontal_tolerance').value
        self.vertical_tolerance = self.get_parameter('vertical_tolerance').value
        self.landing_marker_id = self.get_parameter('landing_marker_id').value
        self.search_descent_speed = self.get_parameter('search_descent_speed').value
        self.precision_horizontal_tolerance = self.get_parameter('precision_horizontal_tolerance').value
        
        # 자유낙하 파라미터 (고도 기반, 단계별)
        self.freefall_altitude_drop = 4.2  # 총 낙하 거리
        self.freefall_stage1_drop = 4.0   # 1단계: 0.0 추력으로 0.8m 하강
        self.freefall_stage2_drop = 0.1    # 2단계: 0.1 추력으로 0.4m 하강  
        self.freefall_stage3_drop = 0.1    # 3단계: 0.3 추력으로 0.8m 하강
        self.stabilization_duration = 3.141592
        
        self.landing_marker_pose = None
        self.last_marker_detection_time = None
        self.precision_landing_start_altitude = None
        self.land_command_issued = False # land 명령 중복 전송 방지 플래그
        
        # 자유낙하 기동을 위한 상태 변수 (고도 기반)
        self.freefall_start_altitude = None  # 자유낙하 시작 고도
        self.stabilization_start_time = None
        
        # 자유낙하 속도 체크 변수 수정 (WP7→WP8)
        self.wp7_arrival_detected = False   # WP7 도착 감지
        self.motors_disabled = False        # 모터 비활성화 여부
        self.velocity_threshold = 0.5       # 속도 임계값 (m/s)
        

        
        # PX4 상태 모니터링 변수
        self.is_offboard_enabled = False    # Offboard 모드 활성화 여부
        self.is_armed = False               # ARM 상태
        
        # 마커 위치 구독자
        self.marker_detection_sub = self.create_subscription(
            Detection3DArray, "/marker_detections", self._marker_detection_callback, 10
        )
        
        # 커맨드 입력 스레드 (간단한 제어용)
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("웨이포인트 미션 및 정밀 착륙 컨트롤러가 초기화되었음")
        self.get_logger().info(f"총 {len(self.drone_waypoints)}개의 웨이포인트가 설정되었음")
        self.get_logger().info(f"착륙 마커 ID: {self.landing_marker_id}, 착륙 고도: {self.landing_altitude}m")
    
    def _land_detected_callback(self, msg: VehicleLandDetected):
        """PX4의 착륙 상태를 감지하는 콜백 (보조용)"""
        # 이 콜백은 이제 주 착륙 감지 메커니즘이 아님
        # check_landed_on_vehicle()가 주된 역할을 함
        if msg.landed and self.state == "LANDING":
             self.get_logger().info("(보조 감지) PX4 컨트롤러가 착륙을 보고했음")
             # 즉시 상태를 변경하지 않고, check_landed_on_vehicle에 의해 처리되도록 둡니다.
             pass

    def _vehicle_control_mode_callback(self, msg: VehicleControlMode):
        """Vehicle Control Mode 콜백 - Offboard 상태 및 ARM 상태 모니터링"""
        self.is_offboard_enabled = msg.flag_control_offboard_enabled
        self.is_armed = msg.flag_armed  # 더 정확한 ARM 상태 체크 (VehicleStatus보다 신뢰성 높음)
        
        # ARM 상태 변경 시 로그 출력
        if hasattr(self, '_last_flag_armed') and self._last_flag_armed != msg.flag_armed:
            self.get_logger().info(f"실제 ARM 상태 변경: {msg.flag_armed}")
        self._last_flag_armed = msg.flag_armed
        
    def _vehicle_status_callback(self, msg: VehicleStatus):
        """Vehicle Status 콜백 - 디버깅용 정보만 수집"""
        # arming_state 값 디버깅 (참고용)
        if msg.arming_state != getattr(self, '_last_arming_state', -1):
            self.get_logger().info(f"VehicleStatus arming_state 변경: {msg.arming_state} (참고용)")
            self._last_arming_state = msg.arming_state
        
        # ARM 상태는 VehicleControlMode의 flag_armed를 사용함 (더 정확함)
        
    def _marker_detection_callback(self, msg: Detection3DArray):
        """마커 탐지 토픽 콜백 함수"""
        if not msg.detections:
            return
            
        # 착륙 마커 ID와 일치하는 마커를 찾음
        for detection in msg.detections:
            if detection.results:
                for result in detection.results:
                    try:
                        marker_id = int(result.hypothesis.class_id)
                        if marker_id == self.landing_marker_id:
                            self.landing_marker_pose = result.pose.pose
                            self.last_marker_detection_time = self.get_clock().now()
                            self.get_logger().debug(f"착륙 마커 {marker_id} 탐지: "
                                                  f"({self.landing_marker_pose.position.x:.2f}, "
                                                  f"{self.landing_marker_pose.position.y:.2f}, "
                                                  f"{self.landing_marker_pose.position.z:.2f})")
                            return
                    except (ValueError, AttributeError) as e:
                        self.get_logger().debug(f"마커 ID 파싱 오류: {e}")
                        continue

    # 미션 컨트롤 연동
    
    def mission_command_callback(self, msg: String):
        """미션 컨트롤 대시보드로부터 명령 수신"""
        command = msg.data.lower()
        
        if command == 'start':
            if self.state == "INIT":
                self.get_logger().info("미션 컨트롤로부터 START 명령 수신. ARM 및 이륙 시작")
                self.start_mission()
            elif self.state == "ARMED_IDLE":
                self.get_logger().info("미션 컨트롤로부터 START 명령 수신. 이미 ARM됨, 바로 이륙 시작")
                self.state = "TAKING_OFF"
            else:
                self.get_logger().warn(f"START 명령을 받았지만 현재 상태가 {self.state}. INIT 또는 ARMED_IDLE 상태에서만 시작 가능")
                
        elif command == 'land':
            if self.state not in ["LANDING", "LANDED"]:
                self.get_logger().info("미션 컨트롤로부터 LAND 명령 수신")
                self.emergency_land()
        
        elif command == 'start_freefall':
            # 더 이상 외부 명령으로 자유낙하를 시작하지 않음 (자동 처리)
            self.get_logger().info("자유낙하는 WP8 도착 시 자동으로 실행됩니다.")

        elif command == 'start_precision_landing':
            if self.state == "AWAITING_LANDING_COMMAND":
                self.get_logger().info("정밀 착륙 시작 명령 수신!")
                self.state = "PRECISION_LANDING"
                # 착륙 시작 고도 기록
                if self.current_map_pose:
                    self.precision_landing_start_altitude = self.current_map_pose.pose.position.z
            else:
                self.get_logger().warn(f"정밀 착륙을 시작할 수 없는 상태: {self.state}")
                
        elif command == 'ugv_arrived':
            self.get_logger().info("UGV 랑데부 도착 신호 수신 - 하강 허가됨")
    
    def send_mission_complete(self, mission_id: int):
        """미션 완료 신호를 미션 컨트롤에 전송"""
        if not self.mission_complete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"미션 컨트롤 서비스를 찾을 수 없음 (ID: {mission_id}) - 서비스 없이 계속 진행")
            return
            
        request = MissionComplete.Request()
        request.mission_id = mission_id
        
        try:
            future = self.mission_complete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"미션 완료 신호 전송 성공 (ID: {mission_id})")
                else:
                    self.get_logger().warn(f"미션 완료 신호 거부됨 (ID: {mission_id}) - 계속 진행")
            else:
                self.get_logger().warn(f"미션 완료 신호 전송 타임아웃 (ID: {mission_id}) - 계속 진행")
                
        except Exception as e:
            self.get_logger().warn(f"미션 완료 신호 전송 실패 (ID: {mission_id}): {e} - 계속 진행")
    
    # 간단한 사용자 입력 처리
    
    def command_input_loop(self):
        """간단한 사용자 명령 처리 루프"""
        print("\n웨이포인트 미션 명령")
        print("  start   - ARM 후 미션 시작")
        print("  land    - 강제 착륙")
        print("--------------------------------")
        
        for line in sys.stdin:
            cmd = line.strip().lower()
            
            if cmd == "start":
                if self.state == "INIT":
                    self.get_logger().info("사용자 명령: START. ARM 후 이륙 시작.")
                    self.start_mission()
                else:
                    self.get_logger().warn(f"START 명령을 사용할 수 없는 상태: {self.state}")
                    
            elif cmd == "land":
                if self.state not in ["LANDING", "LANDED"]:
                    self.get_logger().warn("사용자 명령: LAND. 강제 착륙.")
                    self.emergency_land()
    
    # 시각화
    
    def _create_header(self, frame_id: str) -> Header:
        """지정된 frame_id로 ROS 메시지 헤더를 생성합니다."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _publish_mission_visuals(self):
        """RViz 시각화를 위한 미션 경로, 웨이포인트, 응시 지점 마커를 게시합니다."""
        marker_array = MarkerArray()

        # 0. 이전 마커 모두 삭제 (DELETEALL 액션)
        delete_marker = Marker(action=Marker.DELETEALL)
        marker_array.markers.append(delete_marker)

        # 1. 미션 경로 시각화
        waypoints = self.drone_waypoints.tolist()
        if len(waypoints) > 1:
            path_header = self._create_header("map")
            current_idx = self.current_waypoint_index

            # 남은 경로 (연한 초록색)
            if current_idx < len(waypoints):
                future_waypoints = waypoints[current_idx:]
                if len(future_waypoints) > 1:
                    future_path = visu.create_mission_path_marker(
                        header=path_header, waypoints=future_waypoints,
                        color=(0.1, 1.0, 0.1), alpha=0.2,
                        ns="future_path", marker_id=1)
                    marker_array.markers.append(future_path)

            # 현재 활성 경로 (진한 초록색)
            if 0 < current_idx < len(waypoints):
                active_waypoints = waypoints[current_idx-1:current_idx+1]
                active_path = visu.create_mission_path_marker(
                    header=path_header, waypoints=active_waypoints,
                    color=(0.1, 1.0, 0.1), alpha=0.8,
                    ns="active_path", marker_id=2)
                marker_array.markers.append(active_path)
            elif current_idx == 0 and len(waypoints) > 0:
                # 이륙 후 첫 웨이포인트로 가는 경로는 아직 '활성'으로 표시하지 않음
                pass

            # 지나온 경로 (연한 회색)
            if current_idx > 0:
                # +1을 하여 현재 웨이포인트까지 포함시켜야 이전 경로가 그려짐
                passed_waypoints = waypoints[:current_idx]
                if len(passed_waypoints) > 1:
                    passed_path = visu.create_mission_path_marker(
                        header=path_header, waypoints=passed_waypoints,
                        color=(0.5, 0.5, 0.5), alpha=0.4,
                        ns="passed_path", marker_id=3)
                    marker_array.markers.append(passed_path)

        # 2. 웨이포인트 시각화
        for i, wp in enumerate(waypoints):
            # 웨이포인트 상태 결정
            if i < self.current_waypoint_index:
                status = "passed"
            elif i == self.current_waypoint_index:
                status = "current"
            else:
                status = "future"

            wp_header = self._create_header("map")
            waypoint_markers = visu.create_waypoint_visual(
                header=wp_header,
                waypoint_id=i,
                position=wp,
                waypoint_status=status,
                text_label=f"WP {i}"
            )
            marker_array.markers.extend(waypoint_markers)

        # 3. 응시 지점(Stare Target) 시각화
        for i, target in enumerate(self.stare_targets):
            target_header = self._create_header("map")
            # 현재 웨이포인트에 해당하는 타겟만 강조
            color = (1.0, 0.0, 0.0) if i == self.current_waypoint_index else (1.0, 0.5, 0.0)
            
            target_markers = visu.create_target_visual(
                header=target_header,
                target_id=i,
                position=target,
                color=color,
                text_label=f"Target {i}"
            )
            marker_array.markers.extend(target_markers)
            
        self.visual_marker_publisher.publish(marker_array)
    
    # 미션 로직 구현 (BaseMissionNode의 추상 메서드)
    
    def run_mission_logic(self):
        """웨이포인트 미션의 상태 머신 로직을 구현"""
        
        # 시각화 마커 퍼블리시
        self._publish_mission_visuals()
        
        # 상태별 Offboard Control Mode 발행 (FREEFALLING은 내부에서 처리, 나머지는 base에서 처리)
        # Position 모드 전환 방지를 위해 ARMED_IDLE에서도 Offboard 신호 전송
        if self.state not in ["LANDING", "LANDED", "INIT", "DISARMED", "FREEFALLING", "HANDSHAKE", "STABILIZING"]:
            dcu.publish_offboard_control_mode(self)
        
        # 미션별 상태 처리
        if self.state == "ARMED_IDLE":
            # 자동 미션이므로 ARMED_IDLE 상태에서 자동으로 이륙 시작
            self.get_logger().info("자동 이륙 시작!")
            self.state = "TAKING_OFF"
            
        elif self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING_TO_WAYPOINT":
            self._handle_moving_to_waypoint_state()
        elif self.state == "FREEFALLING":
            self._handle_freefall_state()
        elif self.state == "STABILIZING":
            self._handle_stabilizing_state()
        elif self.state == "AWAITING_LANDING_COMMAND":
            self._handle_awaiting_landing_command_state()
        elif self.state == "PRECISION_LANDING":
            self._handle_precision_landing_state()
        elif self.state == "LANDING":
            # LANDING 상태에서는 UGV 위 착륙을 감지할 때까지 대기
            self.get_logger().info("UGV 위 착륙 감지 확인 중...", throttle_duration_sec=2.0)
            self.check_landed_on_vehicle()
    
    def _handle_takeoff_state(self):
        """이륙 상태 처리"""
        if self.current_map_pose:
            takeoff_altitude = 10.0
            target_pos = [
                self.current_map_pose.pose.position.x,
                self.current_map_pose.pose.position.y,
                takeoff_altitude
            ]
            self.publish_position_setpoint(target_pos)
            
            if abs(self.current_map_pose.pose.position.z - takeoff_altitude) < 3.5:
                self.get_logger().info(f"이륙 완료. 첫 번째 웨이포인트 {self.current_waypoint_index}로 바로출동!")
                # 미션 컨트롤에 이륙 완료 신호 전송
                self.send_mission_complete(4) # DRONE_TAKEOFF_COMPLETE
                self.state = "MOVING_TO_WAYPOINT"
    
    def _handle_moving_to_waypoint_state(self):
        """웨이포인트로 이동 상태 처리 (호버링 없음)"""
        if self.current_waypoint_index >= len(self.drone_waypoints):
            self.get_logger().info("모든 웨이포인트 방문 완료. 최종 지점에서 착륙 명령 대기")
            self.state = "AWAITING_LANDING_COMMAND"
            self.send_mission_complete(5)  # DRONE_APPROACH_COMPLETE
            return

        target_wp_index = self.current_waypoint_index
        target_wp = self.drone_waypoints[target_wp_index]
        target_stare_idx = self.stare_indices[target_wp_index]
        target_stare_pos = self.stare_targets[target_stare_idx]
        
        # WP7 도착 후 속도 체크 로직
        if self.wp7_arrival_detected and target_wp_index == 7:
            # 드론의 XYZ 속도 체크
            if self.current_local_pos:
                # NED 좌표계에서 속도 벡터 크기 계산
                vx = self.current_local_pos.vx
                vy = self.current_local_pos.vy
                vz = self.current_local_pos.vz
                total_velocity = math.sqrt(vx*vx + vy*vy + vz*vz)
                
                self.get_logger().info(f"WP7 속도 체크: {total_velocity:.2f} m/s (임계값: {self.velocity_threshold} m/s)", 
                                     throttle_duration_sec=0.3)
                
                if total_velocity <= self.velocity_threshold:
                    # Offboard 모드 및 ARM 상태 확인
                    if self.is_offboard_enabled and self.is_armed:
                        self.get_logger().info("속도가 임계값 이하로 감소! 자유낙하 시작!")
                        # WP7에서 자유낙하 시작 시 미션 완료 신호 전송
                        self.send_mission_complete(7) # DRONE_WP8_ARRIVAL (자유낙하 시작 신호)
                        self.state = "FREEFALLING"
                        # 자유낙하 시작 고도 기록
                        if self.current_map_pose:
                            self.freefall_start_altitude = self.current_map_pose.pose.position.z
                        # 자유낙하 시작과 함께 짐벌을 아래로 고정
                        dcu.point_gimbal_down(self)
                        self.motors_disabled = False  # 플래그 초기화
                        return
                    else:
                        arming_state_val = getattr(self, '_last_arming_state', 'Unknown')
                        self.get_logger().warn(f"자유낙하 조건 미충족: Offboard={self.is_offboard_enabled}, Armed={self.is_armed} (arming_state={arming_state_val}) - 2이면 Armed, 1이면 Disarmed")
                        # 조건이 맞지 않으면 계속 호버링
                        self.publish_waypoint_setpoint(target_wp_index)
                        self.point_gimbal_at_target(target_stare_pos)
                        return
            
            # 속도가 아직 높으면 현재 위치에서 호버링 유지
            self.publish_waypoint_setpoint(target_wp_index)
            self.point_gimbal_at_target(target_stare_pos)
            return
        
        # 웨이포인트로 이동 (위치 + yaw 제어)
        self.publish_waypoint_setpoint(target_wp_index)
        
        # 주시 타겟 응시
        self.point_gimbal_at_target(target_stare_pos)

        # 도착 확인
        if self.check_arrival(target_wp.tolist(), tolerance=3.5):
            # WP7에 도착한 경우, 속도가 0.5m/s 이하가 될 때까지 대기
            if target_wp_index == 7:
                self.get_logger().info(f"웨이포인트 {target_wp_index} 도착. 속도 체크 시작!")
                self.wp7_arrival_detected = True
                # WP7 도착 시에는 미션 완료 신호를 보내지 않음 (자유낙하 시작 시 전송)
                # 상태는 MOVING_TO_WAYPOINT를 유지하여 속도 체크 로직 실행
                return
            
            self.get_logger().info(f"웨이포인트 {target_wp_index} 통과.")
            
            self.current_waypoint_index += 1

            # 마지막 웨이포인트였는지 확인
            if self.current_waypoint_index >= len(self.drone_waypoints):
                self.get_logger().info("모든 웨이포인트 방문 완료. 최종 지점에서 착륙 명령 대기")
                self.state = "AWAITING_LANDING_COMMAND"
                self.send_mission_complete(5)  # DRONE_APPROACH_COMPLETE
            else:
                # 다음 웨이포인트로 계속 진행 (상태는 MOVING_TO_WAYPOINT 유지)
                self.get_logger().info(f"다음 웨이포인트로 이동: {self.current_waypoint_index}")
    
    def _handle_stabilizing_state(self):
        """안정화 상태. 1.5초 호버링 후 다음 단계로 진행"""
        # 모터 강제 재활성화 및 Offboard Control Mode 발행
        dcu.publish_offboard_control_mode(self)
        
        # 현재 위치에서 호버링
        if self.current_map_pose:
            current_pos = [
                self.current_map_pose.pose.position.x,
                self.current_map_pose.pose.position.y,
                self.current_map_pose.pose.position.z
            ]
            self.publish_position_setpoint(current_pos)
        
        # 안정화 시간 확인
        elapsed_time = (self.get_clock().now() - self.stabilization_start_time).nanoseconds / 1e9
        self.get_logger().info(f"안정화 중... ({elapsed_time:.1f}/{self.stabilization_duration}s)", 
                              throttle_duration_sec=0.5)
        
        if elapsed_time >= self.stabilization_duration:
            self.get_logger().info("안정화 완료. 다음 단계로 진행합니다.")
            
            # 자유낙하 후 안정화만 처리 (WP7→WP8)
            if self.current_waypoint_index == 7:  # 자유낙하는 WP7에서 시작
                self.current_waypoint_index = 8  # WP8로 이동
                self.wp7_arrival_detected = False  # 플래그 리셋
                self.freefall_start_altitude = None  # 자유낙하 고도 리셋
            
            self.state = "MOVING_TO_WAYPOINT"

    def _handle_freefall_state(self):
        """자유낙하 기동 상태. 모터를 강제로 끄고 3.89m 하강 후 다시 켜기"""
        # Offboard failsafe 방지를 위해 지속적으로 offboard control mode 전송
        dcu.publish_offboard_control_mode(self, 
                                          position=False, 
                                          velocity=False, 
                                          acceleration=False,
                                          attitude=False, 
                                          body_rate=False,
                                          thrust_and_torque=False,
                                          actuator=True)
        
        # 모터가 아직 꺼지지 않았다면 강제로 끄기
        if not self.motors_disabled:
            self.get_logger().info("모터 강제 비활성화! 자유낙하 시작")
            self.motors_disabled = True

        # 고도 확인 및 단계별 모터 제어 - 100Hz로 실시간 체크
        if self.current_map_pose and self.freefall_start_altitude is not None:
            current_altitude = self.current_map_pose.pose.position.z
            altitude_dropped = self.freefall_start_altitude - current_altitude
            
            # 단계별 모터 출력 결정
            if altitude_dropped < self.freefall_stage1_drop:
                # 1단계: 0.0 추력으로 0.8m 하강
                motor_output = 0.0
                stage_info = f"1단계 (0.0 추력): {altitude_dropped:.2f}/{self.freefall_stage1_drop}m"
            elif altitude_dropped < (self.freefall_stage1_drop + self.freefall_stage2_drop):
                # 2단계: 0.1 추력으로 0.4m 하강
                motor_output = 0.1
                stage_info = f"2단계 (0.1 추력): {altitude_dropped:.2f}/{self.freefall_stage1_drop + self.freefall_stage2_drop}m"
            elif altitude_dropped < self.freefall_altitude_drop:
                # 3단계: 0.3 추력으로 0.8m 하강
                motor_output = 0.3
                stage_info = f"3단계 (0.3 추력): {altitude_dropped:.2f}/{self.freefall_altitude_drop}m"
            else:
                motor_output = 0.0  # 안전장치
                stage_info = f"완료: {altitude_dropped:.2f}m"
            
            # 모터 출력 전송
            dcu.publish_actuator_motors(self, [motor_output, motor_output, motor_output, motor_output])
            
            # 100Hz 고속 체크, 로그만 5Hz로 출력
            if not hasattr(self, '_freefall_log_counter'):
                self._freefall_log_counter = 0
            self._freefall_log_counter += 1
            
            # 20카운트마다 로그 출력 (100Hz / 20 = 5Hz)
            if self._freefall_log_counter % 20 == 0:
                self.get_logger().info(f"자유낙하 {stage_info} | Offboard: {self.is_offboard_enabled}")
            
            # 실제 조건 체크는 매 100Hz마다 수행 (즉시 반응)
            if altitude_dropped >= self.freefall_altitude_drop:
                self.get_logger().info(f"자유낙하 완료 ({altitude_dropped:.2f}m 하강). 모터 강제 재활성화 후 안정화 모드로 전환")
                self.state = "STABILIZING"
                self.stabilization_start_time = self.get_clock().now()
                self.motors_disabled = False  # 모터 다시 활성화 표시
                self._freefall_log_counter = 0  # 카운터 리셋
        else:
            # 고도 정보가 없으면 기본값
            dcu.publish_actuator_motors(self, [0.0, 0.0, 0.0, 0.0])
            self.get_logger().warn("고도 정보가 없어 자유낙하를 정상적으로 처리할 수 없습니다.", throttle_duration_sec=1.0)

    def _handle_awaiting_landing_command_state(self):
        """착륙 명령 대기 상태 처리"""
        # 마지막 웨이포인트에서 계속 호버링
        final_wp_index = len(self.drone_waypoints) - 1
        self.publish_waypoint_setpoint(final_wp_index)
        
        self.get_logger().info("⏳ 최종 지점에서 호버링하며 착륙 명령 대기 중...", throttle_duration_sec=10.0)

    def _handle_precision_landing_state(self):
        """정밀 착륙 상태 처리 - 마커 탐색 및 정렬 기능 포함"""
        if not self.current_map_pose:
            self.get_logger().warn("현재 위치 정보가 없습니다. 정밀 착륙을 진행할 수 없습니다.")
            return

        # land 명령이 이미 보내졌다면, PX4가 제어권을 가지므로 더 이상 setpoint를 보내지 않음
        if self.land_command_issued:
            return

        current_pos = self.current_map_pose.pose.position

        # 첫 진입 시 짐벌을 아래로 향하게 설정
        if self.precision_landing_start_altitude is not None:
            dcu.point_gimbal_down(self)
            self.get_logger().info("정밀 착륙 모드 시작 - 짐벌을 아래로 향하게 설정")
            self.precision_landing_start_altitude = None  # 한 번만 실행

        # 마커 탐지 여부 확인 (2초 이내)
        marker_detected = (self.landing_marker_pose is not None and
                          self.last_marker_detection_time is not None and
                          (self.get_clock().now() - self.last_marker_detection_time).nanoseconds / 1e9 < 2.0)

        if not marker_detected:
            # 마커 미탐지: 마지막 웨이포인트 위치에서 하강하며 탐색
            self.get_logger().info(f"마커를 찾을 수 없습니다. 고도를 낮추며 탐색합니다. (속도: {self.search_descent_speed} m/s)",
                                 throttle_duration_sec=2.0)

            target_pos = [
                current_pos.x,
                current_pos.y,
                max(self.landing_altitude, current_pos.z - self.search_descent_speed * 0.1) # 10Hz 제어 가정
            ]
            self.publish_position_setpoint(target_pos)
            return

        # --- 마커 탐지됨 ---
        # multi_tracker가 'map' 프레임 기준으로 마커의 절대 좌표를 발행해줌
        marker_world_pos = self.landing_marker_pose.position
        
        h_error = math.sqrt((marker_world_pos.x - current_pos.x)**2 + (marker_world_pos.y - current_pos.y)**2)
        relative_altitude = current_pos.z - marker_world_pos.z

        # 최종 착륙 조건: 마커와의 상대 고도가 1m 미만이고, 수평 오차가 허용치 이내일 때
        if relative_altitude < 1.0 and h_error < self.precision_horizontal_tolerance:
            self.get_logger().info(f"최종 착륙 조건 만족 (상대고도: {relative_altitude:.2f}m, 수평오차: {h_error:.2f}m). PX4 자동 착륙 시작.")
            dcu.land_drone(self)
            self.land_command_issued = True # land 명령 중복 전송 방지
            self.state = "LANDING" # 상태를 LANDING으로 변경
            return

        # 위 조건이 만족되지 않으면, 계속해서 마커를 향해 정렬하며 하강
        target_altitude = max(self.landing_altitude, current_pos.z - self.descent_speed * 0.1)
        target_pos = [marker_world_pos.x, marker_world_pos.y, target_altitude]

        self.publish_position_setpoint(target_pos)
        self.get_logger().info(f"마커 정렬 및 하강 - 상대고도: {relative_altitude:.2f}m, 수평오차: {h_error:.3f}m",
                             throttle_duration_sec=1.0)
    
    # --- 오버라이드 메서드 ---
    
    def on_mission_complete(self):
        """미션 완료 시 추가 처리"""
        if self.state == "MISSION_COMPLETE":
            return # 중복 호출 방지
            
        super().on_mission_complete()
        self.get_logger().info("웨이포인트 미션이 성공적으로 완료되었습니다!")
        self.send_mission_complete(6) # DRONE_HOVER_COMPLETE (미션 완료 신호)


def main(args=None):
    rclpy.init(args=args)
    mission_node = WaypointMissionNode()
    
    try:
        rclpy.spin(mission_node)
        
    except (KeyboardInterrupt, SystemExit):
        mission_node.get_logger().info("시스템 종료 요청. 강제 착륙.")
        if hasattr(mission_node, 'state') and mission_node.state not in ["LANDED", "INIT"]:
            dcu.land_drone(mission_node)
    finally:
        if rclpy.ok():
            mission_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main() 