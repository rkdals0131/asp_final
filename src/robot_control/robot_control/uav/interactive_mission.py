#!/usr/bin/env python3
"""
사용자 대화형 드론 미션 노드
사용자 입력에 따라 드론을 제어하는 오프보드 제어 노드입니다.
- 이륙, 착륙, 정지, 재시동, 지점 이동, 고도 및 짐벌 변경, 특정 지점 응시(stare) 기능을 수행합니다.
"""
import rclpy
import threading
import sys
import copy
import math

from px4_msgs.msg import VehicleLandDetected, GimbalDeviceAttitudeStatus, VehicleCommand, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped

from .base_mission_node import BaseMissionNode
from ..utils import drone_control_utils as dcu
from ..utils import visualization_utils as visu


class InteractiveMissionNode(BaseMissionNode):
    """
    사용자 대화형 미션을 수행하는 노드.
    터미널 입력을 통해 드론을 실시간으로 제어할 수 있습니다.
    """
    
    def __init__(self):
        super().__init__('interactive_mission_node', drone_frame_id="x500_gimbal_0")
        
        # --- 추가 서브스크라이버 (대화형 미션 전용) ---
        self.land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self.land_detected_callback, self.qos_profile
        )
        self.gimbal_status_subscriber = self.create_subscription(
            GimbalDeviceAttitudeStatus, "/fmu/out/gimbal_device_attitude_status",
            self.gimbal_status_callback, self.qos_profile
        )
        
        # marker_publisher 제거 - BaseMissionNode의 visual_marker_publisher 사용
        
        # --- 미션별 상태 변수 ---
        self.land_detected = None
        self.target_pose_map = PoseStamped()
        self.target_pose_map.header.frame_id = 'map'
        self.takeoff_altitude = 10.0
        self.takeoff_target_local = None
        
        # --- Stare 기능을 위한 상태 변수 ---
        self.stare_target_index = None
        
        # --- 목적지 좌표 (ENU) ---
        self.waypoints = [
            [-94.4088, 68.4708, 3.8531],     # 0번
            [-75.4421, 74.9961, 23.2347],    # 1번
            [-65.0308, 80.1275, 8.4990],     # 2번
            [-82.7931, 113.4203, 3.8079],    # 3번
            [-97.9238, 105.2799, 8.5504],    # 4번
            [-109.1330, 100.3533, 23.1363]   # 5번
        ]
        self.final_destination = [-62.9630, 99.0915, 0.1349]  # 'final' 목적지
        
        # --- 사용자 입력을 위한 스레드 ---
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("🎮 대화형 미션 컨트롤러가 실행 중입니다.")
        self.get_logger().info("TF 및 Local Position 데이터를 기다리는 중...")
        self.get_logger().info("💡 'start' 또는 'arm' 명령으로 드론을 시동하세요.")
    
    # --- 추가 콜백 함수들 ---
    
    def land_detected_callback(self, msg: VehicleLandDetected):
        """착륙 감지 콜백"""
        self.land_detected = msg
    
    def gimbal_status_callback(self, msg: GimbalDeviceAttitudeStatus):
        """짐벌 상태 콜백"""
        q = msg.q
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(
            f"짐벌 상태 -> Pitch: {math.degrees(pitch):.1f}°, Yaw: {math.degrees(yaw):.1f}°",
            throttle_duration_sec=1
        )
    
    # --- 사용자 입력 처리 ---
    
    def command_input_loop(self):
        """사용자 명령을 터미널에서 입력받는 루프."""
        print("\n--- 드론 명령 콘솔 ---")
        print("  [비행 제어]")
        print("    start                    - 드론 시동 및 ARM (처음)")
        print("    takeoff                  - 이륙")
        print("    land                     - 착륙")
        print("    arm                      - (착륙 후) 재시동")
        print("    stop                     - (비행 중) 현재 위치에 정지")
        print("  [이동 제어]")
        print("    go <0-5|final>           - 지정 목적지로 이동 (고도 변경 포함)")
        print("    strafe <0-5|final>       - 지정 목적지로 수평 이동 (현재 고도 유지)")
        print("    climb <meters>           - 지정한 미터만큼 상대 고도 상승")
        print("    descend <meters>         - 지정한 미터만큼 상대 고도 하강")
        print("    maintain <altitude>      - 지정한 절대 고도로 이동/유지")
        print("    moveto <x> <y> <z>       - 지정한 절대좌표(map frame)로 이동")
        print("  [짐벌 제어]")
        print("    look <0-5>               - 지정 번호의 마커를 한번 바라봄")
        print("    look forward             - 짐벌 정면으로 초기화")
        print("    look down                - 짐벌 수직 아래로")
        print("    stare <0-5>              - 지정 번호의 마커를 계속 추적/응시")
        print("    stare stop               - 추적/응시 중지")
        print("-----------------------------")

        # 명령어-핸들러 매핑
        single_commands = {
            "start": self._handle_start_command,
            "takeoff": self._handle_takeoff_command,
            "land": self._handle_land_command,
            "arm": self._handle_arm_command,
            "stop": self._handle_stop_command,
        }
        
        for line in sys.stdin:
            cmd = line.strip().split()
            if not cmd:
                continue
            command = cmd[0].lower()
            
            # 단일 명령어 처리
            if command in single_commands:
                single_commands[command]()
            
            # 인자가 필요한 명령어들
            elif command in ["go", "strafe"] and len(cmd) > 1:
                self._handle_move_command(command, cmd[1])
            elif command in ["climb", "descend"] and len(cmd) > 1:
                self._handle_altitude_change_command(command, cmd[1])
            elif command == "maintain" and len(cmd) > 1:
                self._handle_maintain_command(cmd[1])
            elif command == "moveto" and len(cmd) > 3:
                self._handle_moveto_command(cmd[1:])
            elif command == "look" and len(cmd) > 1:
                self._handle_look_command(cmd[1])
            elif command == "stare" and len(cmd) > 1:
                self._handle_stare_command(cmd[1])
            else:
                self.get_logger().warn(f"알 수 없는 명령: '{line.strip()}'")
    
    def _handle_start_command(self):
        if self.state == "INIT":
            self.get_logger().info("사용자 명령: START. 드론 시동 및 ARM.")
            self.start_mission()
        else:
            self.get_logger().warn(f"START 명령을 사용할 수 없는 상태입니다: {self.state}")
    
    def _handle_takeoff_command(self):
        if self.state == "ARMED_IDLE":
            self.state = "TAKING_OFF"
        else:
            self.get_logger().warn(f"이륙할 수 없는 상태입니다: {self.state}")
    
    def _handle_land_command(self):
        if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
            self.state = "LANDING"
        else:
            self.get_logger().warn(f"착륙할 수 없는 상태입니다: {self.state}")
    
    def _handle_arm_command(self):
        if self.state == "LANDED":
            self.get_logger().info("사용자 명령: ARM. 드론 재시동.")
            dcu.arm_and_offboard(self)
            self.state = "ARMED_IDLE"
        else:
            self.get_logger().warn(f"ARM은 LANDED 상태에서만 가능합니다. 현재 상태: {self.state}")
    
    def _handle_stop_command(self):
        if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
            if self.current_map_pose:
                self.get_logger().info("사용자 명령: STOP. 이동 정지.")
                self.target_pose_map = copy.deepcopy(self.current_map_pose)
                self.state = "IDLE"
            else:
                self.get_logger().warn("현재 위치를 알 수 없어 정지할 수 없습니다.")
        else:
            self.get_logger().warn(f"정지할 수 없는 상태입니다: {self.state}")
    
    def _handle_move_command(self, command, target_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"'{command}' 명령을 실행할 수 없는 상태입니다: {self.state}")
                return
            
            if target_str == "final":
                wp = self.final_destination
                self.get_logger().info(f"사용자 명령: {command.upper()} to final destination.")
            else:
                wp_index = int(target_str)
                wp = self.waypoints[wp_index]
                self.get_logger().info(f"사용자 명령: {command.upper()} to waypoint {wp_index}.")
            
            self.target_pose_map.pose.position.x = wp[0]
            self.target_pose_map.pose.position.y = wp[1]
            
            if command == "go":
                self.target_pose_map.pose.position.z = wp[2]
            elif command == "strafe":
                self.target_pose_map.pose.position.z = self.current_map_pose.pose.position.z
            
            self.state = "MOVING"
            
        except (ValueError, IndexError):
            self.get_logger().error(f"잘못된 웨이포인트 인덱스입니다. 0-{len(self.waypoints)-1} 또는 'final'을 사용하세요")
    
    def _handle_altitude_change_command(self, command, value_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"고도 변경을 할 수 없는 상태입니다: {self.state}")
                return
            
            alt_change = float(value_str) * (-1 if command == "descend" else 1)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z += alt_change
            
            self.get_logger().info(f"상대 고도 변경 {alt_change:+.1f}m. 새 목표 Z: {self.target_pose_map.pose.position.z:.2f}m")
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"잘못된 고도 값입니다: {value_str}")
    
    def _handle_maintain_command(self, value_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"고도 유지를 할 수 없는 상태입니다: {self.state}")
                return
            
            target_alt = float(value_str)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z = target_alt
            
            self.get_logger().info(f"절대 고도 {target_alt:.2f}m로 유지합니다.")
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"잘못된 고도 값입니다: {value_str}")
    
    def _handle_moveto_command(self, args):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"'moveto' 명령을 실행할 수 없는 상태입니다: {self.state}")
                return
            
            if len(args) != 3:
                self.get_logger().error("moveto 명령은 3개의 인자가 필요합니다: x, y, z")
                return
            
            x, y, z = float(args[0]), float(args[1]), float(args[2])
            
            self.target_pose_map.pose.position.x = x
            self.target_pose_map.pose.position.y = y
            self.target_pose_map.pose.position.z = z
            
            self.get_logger().info(f"사용자 명령: MOVETO to ({x:.2f}, {y:.2f}, {z:.2f}).")
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"moveto에 잘못된 좌표입니다. 숫자 3개를 입력하세요.")
    
    def _handle_look_command(self, sub_command):
        self.stare_target_index = None  # 'look' 명령은 항상 'stare'를 중지시킴
        sub_command = sub_command.lower()
        
        if sub_command == "forward":
            self.get_logger().info("사용자 명령: LOOK FORWARD")
            dcu.reset_gimbal(self)
        elif sub_command == "down":
            self.get_logger().info("사용자 명령: LOOK DOWN")
            dcu.point_gimbal_down(self)
        else:
            try:
                target_index = int(sub_command)
                if 0 <= target_index < len(self.waypoints):
                    self.get_logger().info(f"사용자 명령: LOOK {target_index}")
                    self.point_gimbal_at_target(self.waypoints[target_index])
                else:
                    self.get_logger().error(f"Look 인덱스 {target_index}가 범위를 벗어났습니다.")
            except ValueError:
                self.get_logger().error(f"잘못된 look 명령입니다. 'forward', 'down', 또는 인덱스 0-{len(self.waypoints)-1}을 사용하세요")
    
    def _handle_stare_command(self, sub_command):
        sub_command = sub_command.lower()
        
        if sub_command == "stop":
            if self.stare_target_index is not None:
                self.get_logger().info("사용자 명령: STARE STOP. 짐벌 추적을 중지합니다.")
                self.stare_target_index = None
            else:
                self.get_logger().info("현재 응시하고 있는 대상이 없습니다.")
        else:
            try:
                target_index = int(sub_command)
                if 0 <= target_index < len(self.waypoints):
                    self.get_logger().info(f"사용자 명령: STARE {target_index}. 웨이포인트를 계속 추적합니다.")
                    self.stare_target_index = target_index
                    # 즉시 한번 조준 실행
                    self.point_gimbal_at_target(self.waypoints[self.stare_target_index])
                else:
                    self.get_logger().error(f"Stare 인덱스 {target_index}가 범위를 벗어났습니다.")
            except ValueError:
                self.get_logger().error(f"잘못된 stare 명령입니다. 'stop' 또는 인덱스 0-{len(self.waypoints)-1}을 사용하세요")
    
    # --- 시각화 ---
    
    def _publish_all_markers(self):
        """모든 웨이포인트와 final_destination 위치에 마커를 게시합니다."""
        marker_array = visu.create_interactive_mission_markers(
            self, self.waypoints, self.final_destination
        )
        self.visual_marker_publisher.publish(marker_array)
    
    # --- 미션 로직 구현 (BaseMissionNode의 추상 메서드) ---
    
    def run_mission_logic(self):
        """대화형 미션의 상태 머신 로직을 구현합니다."""
        
        # Stare 모드 실행
        if self.stare_target_index is not None and self.state in ["IDLE", "MOVING"]:
            self.point_gimbal_at_target(self.waypoints[self.stare_target_index])
        
        # 시각화 마커 퍼블리시
        self._publish_all_markers()
        
        # 미션별 상태 처리
        if self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING":
            self._handle_moving_state()
        elif self.state == "IDLE":
            self._handle_idle_state()
        elif self.state == "LANDING":
            self._handle_landing_state()
    
    def _handle_takeoff_state(self):
        """이륙 상태 처리"""
        if self.takeoff_target_local is None:
            self.get_logger().info(f"이륙 시작. 목표 고도: {self.takeoff_altitude}m.")
            self.takeoff_target_local = [
                self.current_local_pos.x,
                self.current_local_pos.y,
                self.current_local_pos.z - self.takeoff_altitude
            ]
        
        # 이륙 세트포인트 퍼블리시
        sp_msg = TrajectorySetpoint(
            position=[float(p) for p in self.takeoff_target_local],
            timestamp=int(self.get_clock().now().nanoseconds / 1000)
        )
        self.trajectory_setpoint_publisher.publish(sp_msg)
        
        # 이륙 완료 확인
        if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 1.0:
            self.get_logger().info("이륙 완료. 호버링 상태.")
            self.target_pose_map = copy.deepcopy(self.current_map_pose)
            self.state = "IDLE"
            self.takeoff_target_local = None
    
    def _handle_moving_state(self):
        """이동 상태 처리"""
        target_pos = [
            self.target_pose_map.pose.position.x,
            self.target_pose_map.pose.position.y,
            self.target_pose_map.pose.position.z
        ]
        self.publish_position_setpoint(target_pos)
        
        if self.check_arrival(target_pos):
            self.get_logger().info("목적지 도착. 호버링 상태.")
            self.state = "IDLE"
    
    def _handle_idle_state(self):
        """대기 상태 처리"""
        target_pos = [
            self.target_pose_map.pose.position.x,
            self.target_pose_map.pose.position.y,
            self.target_pose_map.pose.position.z
        ]
        self.publish_position_setpoint(target_pos)
    
    def _handle_landing_state(self):
        """착륙 상태 처리"""
        if self.land_detected and self.land_detected.landed:
            self.get_logger().info("착륙 성공. 'arm' 명령으로 재시동할 수 있습니다.")
            self.state = "LANDED"


def main(args=None):
    rclpy.init(args=args)
    mission_node = InteractiveMissionNode()
    
    try:
        rclpy.spin(mission_node)
        
    except (KeyboardInterrupt, SystemExit):
        mission_node.get_logger().info("종료 요청. 강제 착륙.")
        if hasattr(mission_node, 'state') and mission_node.state not in ["LANDED", "INIT"]:
            dcu.land_drone(mission_node)
    finally:
        if rclpy.ok():
            mission_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main() 