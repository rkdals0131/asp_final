#!/usr/bin/env python3
"""
웨이포인트 자동 순회 및 정밀 착륙 미션 노드
지정된 웨이포인트를 순서대로 방문하고, 마지막 지점에서 아루코 마커를 이용해 정밀 착륙을 수행
"""

import rclpy
import threading
import sys
import math

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from px4_msgs.msg import VehicleLandDetected  # 착륙 감지 메시지
from mission_admin_interfaces.srv import MissionComplete
from rcl_interfaces.msg import ParameterDescriptor

from .base_mission_node import BaseMissionNode
from ..utils import drone_control_utils as dcu
from ..utils import visualization_utils as visu


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
        
        # 미션 컨트롤 서비스 클라이언트
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')
        
        # 웨이포인트 미션 관련 변수
        self.current_waypoint_index = 0
        
        # 짐벌 카메라 프레임 ID 파라미터로 로드 (config에서 받아옴)
        self.declare_parameter('gimbal_camera_frame', 'x500_gimbal_0/camera_link')
        self.gimbal_camera_frame_id = self.get_parameter('gimbal_camera_frame').value
        
        # 정밀 착륙 관련 변수 및 파라미터
        self.declare_parameter('landing_altitude', 0.5)
        self.declare_parameter('descent_speed', 7.0, 
            ParameterDescriptor(description="마커 정렬 후 최종 착륙 시 하강 속도 (m/s)"))
        self.declare_parameter('horizontal_tolerance', 0.15)
        self.declare_parameter('vertical_tolerance', 0.3)
        self.declare_parameter('landing_marker_id', 6)  # 착륙용 마커 ID
        self.declare_parameter('search_descent_speed', 7.0, 
            ParameterDescriptor(description="마커를 탐색하며 하강할 때의 속도 (m/s)"))
        self.declare_parameter('precision_horizontal_tolerance', 0.1)  # 정밀 착륙 시 수평 허용 오차
        
        self.landing_altitude = self.get_parameter('landing_altitude').value
        self.descent_speed = self.get_parameter('descent_speed').value
        self.horizontal_tolerance = self.get_parameter('horizontal_tolerance').value
        self.vertical_tolerance = self.get_parameter('vertical_tolerance').value
        self.landing_marker_id = self.get_parameter('landing_marker_id').value
        self.search_descent_speed = self.get_parameter('search_descent_speed').value
        self.precision_horizontal_tolerance = self.get_parameter('precision_horizontal_tolerance').value
        
        self.landing_marker_pose = None
        self.last_marker_detection_time = None
        self.precision_landing_start_altitude = None
        self.land_command_issued = False # land 명령 중복 전송 방지 플래그
        
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
        self.get_logger().info(f"🎯 착륙 마커 ID: {self.landing_marker_id}, 착륙 고도: {self.landing_altitude}m")
    
    def _land_detected_callback(self, msg: VehicleLandDetected):
        """PX4의 착륙 상태를 감지하는 콜백 (보조용)"""
        # 이 콜백은 이제 주 착륙 감지 메커니즘이 아님
        # check_landed_on_vehicle()가 주된 역할을 함
        if msg.landed and self.state == "LANDING":
             self.get_logger().info("(보조 감지) PX4 컨트롤러가 착륙을 보고했음")
             # 즉시 상태를 변경하지 않고, check_landed_on_vehicle에 의해 처리되도록 둡니다.
             pass

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
    
    def _publish_mission_visuals(self):
        """RViz 시각화를 위한 모든 마커(경로, 타겟, 짐벌 방향)를 생성하고 게시합니다."""
        marker_array = visu.create_mission_visual_markers(
            self, 
            self.drone_waypoints.tolist(), 
            self.stare_targets,
            self.current_waypoint_index
        )
        self.visual_marker_publisher.publish(marker_array)
    
    # 미션 로직 구현 (BaseMissionNode의 추상 메서드)
    
    def run_mission_logic(self):
        """웨이포인트 미션의 상태 머신 로직을 구현"""
        
        # 시각화 마커 퍼블리시
        self._publish_mission_visuals()
        
        # 미션별 상태 처리
        if self.state == "ARMED_IDLE":
            # 자동 미션이므로 ARMED_IDLE 상태에서 자동으로 이륙 시작
            self.get_logger().info("자동 이륙 시작!")
            self.state = "TAKING_OFF"
            
        elif self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING_TO_WAYPOINT":
            self._handle_moving_to_waypoint_state()
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
            takeoff_altitude = 5.0
            target_pos = [
                self.current_map_pose.pose.position.x,
                self.current_map_pose.pose.position.y,
                takeoff_altitude
            ]
            self.publish_position_setpoint(target_pos)
            
            if abs(self.current_map_pose.pose.position.z - takeoff_altitude) < 1.0:
                self.get_logger().info(f"이륙 완료. 첫 번째 웨이포인트 {self.current_waypoint_index}로 이동")
                # 미션 컨트롤에 이륙 완료 신호 전송
                self.send_mission_complete(2) # DRONE_TAKEOFF_COMPLETE
                self.state = "MOVING_TO_WAYPOINT"
    
    def _handle_moving_to_waypoint_state(self):
        """웨이포인트로 이동 상태 처리 (호버링 없음)"""
        if self.current_waypoint_index >= len(self.drone_waypoints):
            self.get_logger().info("모든 웨이포인트 방문 완료. 최종 지점에서 착륙 명령 대기")
            self.state = "AWAITING_LANDING_COMMAND"
            self.send_mission_complete(4)  # DRONE_APPROACH_COMPLETE
            return

        target_wp = self.drone_waypoints[self.current_waypoint_index]
        target_stare_idx = self.stare_indices[self.current_waypoint_index]
        target_stare_pos = self.stare_targets[target_stare_idx]
        
        # 웨이포인트로 이동 (위치 + yaw 제어)
        self.publish_waypoint_setpoint(self.current_waypoint_index)
        
        # 주시 타겟 응시
        self.point_gimbal_at_target(target_stare_pos)

        # 도착 확인
        if self.check_arrival(target_wp.tolist(), tolerance=3.5):
            self.get_logger().info(f"웨이포인트 {self.current_waypoint_index} 통과.")
            self.current_waypoint_index += 1

            # 마지막 웨이포인트였는지 확인
            if self.current_waypoint_index >= len(self.drone_waypoints):
                self.get_logger().info("모든 웨이포인트 방문 완료. 최종 지점에서 착륙 명령 대기")
                self.state = "AWAITING_LANDING_COMMAND"
                self.send_mission_complete(4)  # DRONE_APPROACH_COMPLETE
            else:
                # 다음 웨이포인트로 계속 진행 (상태는 MOVING_TO_WAYPOINT 유지)
                self.get_logger().info(f"다음 웨이포인트로 이동: {self.current_waypoint_index}")
    
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
            self.get_logger().info("🎯 정밀 착륙 모드 시작 - 짐벌을 아래로 향하게 설정")
            self.precision_landing_start_altitude = None  # 한 번만 실행

        # 마커 탐지 여부 확인 (2초 이내)
        marker_detected = (self.landing_marker_pose is not None and
                          self.last_marker_detection_time is not None and
                          (self.get_clock().now() - self.last_marker_detection_time).nanoseconds / 1e9 < 2.0)

        if not marker_detected:
            # 마커 미탐지: 마지막 웨이포인트 위치에서 하강하며 탐색
            self.get_logger().info(f"🔍 마커를 찾을 수 없습니다. 고도를 낮추며 탐색합니다. (속도: {self.search_descent_speed} m/s)",
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
            self.get_logger().info(f"🛬 최종 착륙 조건 만족 (상대고도: {relative_altitude:.2f}m, 수평오차: {h_error:.2f}m). PX4 자동 착륙 시작.")
            dcu.land_drone(self)
            self.land_command_issued = True # land 명령 중복 전송 방지
            self.state = "LANDING" # 상태를 LANDING으로 변경
            return

        # 위 조건이 만족되지 않으면, 계속해서 마커를 향해 정렬하며 하강
        target_altitude = max(self.landing_altitude, current_pos.z - self.descent_speed * 0.1)
        target_pos = [marker_world_pos.x, marker_world_pos.y, target_altitude]

        self.publish_position_setpoint(target_pos)
        self.get_logger().info(f"🎯 마커 정렬 및 하강 - 상대고도: {relative_altitude:.2f}m, 수평오차: {h_error:.3f}m",
                             throttle_duration_sec=1.0)
    
    # --- 오버라이드 메서드 ---
    
    def on_mission_complete(self):
        """미션 완료 시 추가 처리"""
        if self.state == "MISSION_COMPLETE":
            return # 중복 호출 방지
            
        super().on_mission_complete()
        self.get_logger().info("🎯 웨이포인트 미션이 성공적으로 완료되었습니다!")
        self.send_mission_complete(5) # DRONE_HOVER_COMPLETE (미션 완료 신호)


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