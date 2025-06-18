#!/usr/bin/env python3
"""
웨이포인트 자동 순회 미션 노드
지정된 웨이포인트를 순서대로 방문하며, 각 지점에서 스타르 타겟을 응시하고 호버링하는 자동 미션을 수행합니다.
"""

import rclpy
import numpy as np
import threading
import sys

from std_msgs.msg import String
from mission_admin_interfaces.srv import MissionComplete

from .base_mission_node import BaseMissionNode
from . import drone_control_utils as dcu
from . import visualization_utils as visu


class WaypointMissionNode(BaseMissionNode):
    """
    웨이포인트 기반 자동 미션을 수행하는 노드.
    지정된 웨이포인트로 이동하며, 각 지점에서 Stare 타겟을 응시하고 2초간 호버링합니다.
    """
    
    def __init__(self):
        super().__init__('waypoint_mission_node')
        
        # --- 추가 서브스크라이버 (미션 컨트롤 연동) ---
        self.mission_command_sub = self.create_subscription(
            String, "/drone/mission_command", self.mission_command_callback, 10
        )
        
        # --- 미션 컨트롤 서비스 클라이언트 ---
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')
        
        # --- 미션 정의 ---
        mission_definition = [
            (-100, 80, 20, 0), (-80, 80, 30, 1), (-65, 83, 25, 2),
            (-80, 100, 20, 3), (-90, 103, 25, 4), (-105, 95, 30, 5),
            (-63, 100, 10, 6)
        ]
        
        self.drone_waypoints = np.array([p[0:3] for p in mission_definition], dtype=np.float64)
        self.stare_indices = np.array([p[3] for p in mission_definition])
        self.stare_targets = [
            [-94.4088, 68.4708, 3.8531], [-75.4421, 74.9961, 23.2347],
            [-65.0308, 80.1275, 8.4990], [-82.7931, 113.4203, 3.8079],
            [-97.9238, 105.2799, 8.5504], [-109.1330, 100.3533, 23.1363],
            [-62.9630, 99.0915, 0.1349]
        ]
        
        # --- 웨이포인트 미션 관련 변수 ---
        self.current_waypoint_index = 0
        self.hover_start_time = None
        self.gimbal_camera_frame_id = "x500_gimbal_0/camera_link"
        
        # --- 커맨드 입력 스레드 (간단한 제어용) ---
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("🛩️ 웨이포인트 미션 컨트롤러가 초기화되었습니다.")
    
    # --- 미션 컨트롤 연동 ---
    
    def mission_command_callback(self, msg: String):
        """미션 컨트롤 대시보드로부터 명령 수신"""
        command = msg.data.lower()
        
        if command == 'start':
            if self.state == "INIT":
                self.get_logger().info("🚁 미션 컨트롤로부터 START 명령 수신. ARM 및 이륙 시작")
                self.start_mission()
            elif self.state == "ARMED_IDLE":
                self.get_logger().info("🚁 미션 컨트롤로부터 START 명령 수신. 이미 ARM됨, 바로 이륙 시작")
                self.state = "TAKING_OFF"
            else:
                self.get_logger().warn(f"START 명령을 받았지만 현재 상태가 {self.state}입니다. INIT 또는 ARMED_IDLE 상태에서만 시작 가능합니다.")
                
        elif command == 'land':
            if self.state not in ["LANDING", "LANDED"]:
                self.get_logger().info("⛔ 미션 컨트롤로부터 LAND 명령 수신")
                self.emergency_land()
    
    def send_mission_complete(self, mission_id: int):
        """미션 완료 신호를 미션 컨트롤에 전송"""
        if not self.mission_complete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"미션 컨트롤 서비스를 찾을 수 없습니다 (ID: {mission_id}) - 서비스 없이 계속 진행")
            return
            
        request = MissionComplete.Request()
        request.mission_id = mission_id
        
        try:
            future = self.mission_complete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"✅ 미션 완료 신호 전송 성공 (ID: {mission_id})")
                else:
                    self.get_logger().warn(f"⚠️ 미션 완료 신호 거부됨 (ID: {mission_id}) - 계속 진행")
            else:
                self.get_logger().warn(f"⚠️ 미션 완료 신호 전송 타임아웃 (ID: {mission_id}) - 계속 진행")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ 미션 완료 신호 전송 실패 (ID: {mission_id}): {e} - 계속 진행")
    
    # --- 간단한 사용자 입력 처리 ---
    
    def command_input_loop(self):
        """간단한 사용자 명령 처리 루프"""
        print("\n--- 웨이포인트 미션 명령 ---")
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
                    self.get_logger().warn(f"START 명령을 사용할 수 없는 상태입니다: {self.state}")
                    
            elif cmd == "land":
                if self.state not in ["LANDING", "LANDED"]:
                    self.get_logger().warn("사용자 명령: LAND. 강제 착륙.")
                    self.emergency_land()
    
    # --- 시각화 ---
    
    def _publish_mission_visuals(self):
        """RViz 시각화를 위한 모든 마커(경로, 타겟, 짐벌 방향)를 생성하고 게시합니다."""
        marker_array = visu.create_mission_visual_markers(
            self, 
            self.drone_waypoints.tolist(), 
            self.stare_targets,
            self.current_waypoint_index
        )
        self.visual_marker_publisher.publish(marker_array)
    
    # --- 미션 로직 구현 (BaseMissionNode의 추상 메서드) ---
    
    def run_mission_logic(self):
        """웨이포인트 미션의 상태 머신 로직을 구현합니다."""
        
        # 시각화 마커 퍼블리시
        self._publish_mission_visuals()
        
        # 미션별 상태 처리
        if self.state == "ARMED_IDLE":
            # 자동 미션이므로 ARMED_IDLE 상태에서 자동으로 이륙 시작
            self.get_logger().info("🚁 자동 이륙 시작!")
            self.state = "TAKING_OFF"
            
        elif self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING_TO_WAYPOINT":
            self._handle_moving_to_waypoint_state()
        elif self.state == "HOVERING_AT_WAYPOINT":
            self._handle_hovering_at_waypoint_state()
        elif self.state == "MISSION_COMPLETE_HOVER":
            self._handle_mission_complete_hover_state()
    
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
                self.get_logger().info(f"🚁 이륙 완료. 첫 번째 웨이포인트 {self.current_waypoint_index}로 이동.")
                # 미션 컨트롤에 이륙 완료 신호 전송
                self.send_mission_complete(2)  # DRONE_TAKEOFF_COMPLETE
                self.state = "MOVING_TO_WAYPOINT"
    
    def _handle_moving_to_waypoint_state(self):
        """웨이포인트로 이동 상태 처리"""
        if self.current_waypoint_index >= len(self.drone_waypoints):
            self.state = "LANDING"
            return

        target_wp = self.drone_waypoints[self.current_waypoint_index]
        target_stare_idx = self.stare_indices[self.current_waypoint_index]
        target_stare_pos = self.stare_targets[target_stare_idx]
        
        # 웨이포인트로 이동
        self.publish_position_setpoint(target_wp.tolist())
        
        # 스타르 타겟 응시
        self.point_gimbal_at_target(target_stare_pos)

        # 도착 확인
        if self.check_arrival(target_wp.tolist()):
            self.get_logger().info(f"웨이포인트 {self.current_waypoint_index} 도착. 2초간 호버링.")
            self.state = "HOVERING_AT_WAYPOINT"
            self.hover_start_time = self.get_clock().now()
    
    def _handle_hovering_at_waypoint_state(self):
        """웨이포인트에서 호버링 상태 처리"""
        if self.hover_start_time is None:
            self.state = "MOVING_TO_WAYPOINT"
            return

        target_wp = self.drone_waypoints[self.current_waypoint_index]
        target_stare_idx = self.stare_indices[self.current_waypoint_index]
        target_stare_pos = self.stare_targets[target_stare_idx]
        
        # 현재 위치 유지
        self.publish_position_setpoint(target_wp.tolist())
        
        # 스타르 타겟 계속 응시
        self.point_gimbal_at_target(target_stare_pos)
        
        # 2초 호버링 완료 확인
        if self.get_clock().now() - self.hover_start_time > rclpy.duration.Duration(seconds=2):
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.drone_waypoints):
                self.get_logger().info("🏁 모든 웨이포인트 방문 완료. 현재 위치에서 호버링 시작.")
                self.state = "MISSION_COMPLETE_HOVER"
                # 미션 컨트롤에 랑데뷰 지점 도착 및 호버링 완료 신호 전송
                self.send_mission_complete(4)  # DRONE_APPROACH_COMPLETE
                self.send_mission_complete(5)  # DRONE_HOVER_COMPLETE
            else:
                self.get_logger().info(f"호버링 완료. 다음 웨이포인트로 이동: {self.current_waypoint_index}")
                self.state = "MOVING_TO_WAYPOINT"
            
            self.hover_start_time = None
    
    def _handle_mission_complete_hover_state(self):
        """미션 완료 후 호버링 상태 처리"""
        # 마지막 웨이포인트에서 계속 호버링 (무한 호버링)
        final_wp = self.drone_waypoints[-1]
        final_stare_idx = self.stare_indices[-1]
        final_stare_pos = self.stare_targets[final_stare_idx]
        
        self.publish_position_setpoint(final_wp.tolist())
        self.point_gimbal_at_target(final_stare_pos)
        
        self.get_logger().info("✈️ 미션 완료 - 마지막 웨이포인트에서 호버링 중...", throttle_duration_sec=10.0)
    
    # --- 오버라이드 메서드 ---
    
    def on_mission_complete(self):
        """미션 완료 시 추가 처리"""
        super().on_mission_complete()
        self.get_logger().info("🎯 웨이포인트 미션이 성공적으로 완료되었습니다!")


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