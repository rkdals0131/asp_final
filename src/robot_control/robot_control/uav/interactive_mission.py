#!/usr/bin/env python3
"""
사용자 대화형 드론 미션 노드
사용자 입력에 따라 드론을 제어하는 오프보드 제어 노드
- 이륙, 착륙, 정지, 재시동, 지점 이동, 고도 및 짐벌 변경, 특정 지점 응시(stare) 기능을 수행
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
    터미널 입력을 통해 드론을 실시간으로 제어할 수 있음
    """
    
    def __init__(self):
        super().__init__('interactive_mission_node', drone_frame_id="x500_gimbal_0")
        
        # 추가 서브스크라이버 (대화형 미션 전용)
        self.land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self.land_detected_callback, self.qos_profile
        )
        self.gimbal_status_subscriber = self.create_subscription(
            GimbalDeviceAttitudeStatus, "/fmu/out/gimbal_device_attitude_status",
            self.gimbal_status_callback, self.qos_profile
        )
        
        # 미션별 상태 변수
        self.land_detected = None
        self.target_pose_map = PoseStamped()
        self.target_pose_map.header.frame_id = 'map'
        self.takeoff_altitude = 10.0
        self.takeoff_target_local = None
        
        # Stare 기능을 위한 상태 변수
        self.stare_target_index = None
        
        # Head 기능을 위한 상태 변수
        self.target_yaw_deg = None
        
        # 사용자 입력을 위한 스레드
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("대화형 미션 컨트롤러가 실행 중")
        self.get_logger().info(f"📍 드론 웨이포인트: {len(self.drone_waypoints)}개, 주시 타겟: {len(self.stare_targets)}개")
        self.get_logger().info("TF 및 Local Position 데이터를 기다리는 중...")
        self.get_logger().info("💡 'start' 또는 'arm' 명령으로 드론을 시동하세요.")
    
    # 추가 콜백 함수들
    
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
    
    # 사용자 입력 처리
    
    def command_input_loop(self):
        """사용자 명령을 터미널에서 입력받는 루프."""
        print("\n드론 명령 콘솔")
        print("  [비행 제어]")
        print("    start                    - 드론 시동 및 ARM (처음)")
        print("    takeoff                  - 이륙")
        print("    land                     - 착륙")
        print("    arm                      - (착륙 후) 재시동")
        print("    stop                     - (비행 중) 현재 위치에 정지")
        print("  [이동 제어]")
        print("    go <0-5|final>           - 주시 타겟으로 이동 (고도 변경 포함)")
        print("    strafe <0-5|final>       - 주시 타겟으로 수평 이동 (현재 고도 유지)")
        print("    go_wp <0-6>              - 드론 웨이포인트로 이동")
        print("    strafe_wp <0-6>          - 드론 웨이포인트로 수평 이동 (현재 고도 유지)")
        print("    climb <meters>           - 지정한 미터만큼 상대 고도 상승")
        print("    descend <meters>         - 지정한 미터만큼 상대 고도 하강")
        print("    maintain <altitude>      - 지정한 절대 고도로 이동/유지")
        print("    moveto <x> <y> <z>       - 지정한 절대좌표(map frame)로 이동")
        print("    head <degrees>           - 현재 위치에서 지정한 각도 방향으로 회전 (0=동쪽, 90=북쪽, 180=서쪽, 270=남쪽)")
        print("  [짐벌 제어]")
        print("    look <0-6>               - 지정 번호의 주시 타겟을 한번 바라봄")
        print("    look forward             - 짐벌 정면으로 초기화")
        print("    look down                - 짐벌 수직 아래로")
        print("    stare <0-6>              - 지정 번호의 주시 타겟을 계속 추적/응시")
        print("    stare stop               - 추적/응시 중지")
        print("    gimbal set <pitch> <yaw> [roll] - 짐벌을 지정 각도로 설정")
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
                self._handle_move_command(command, cmd[1], use_stare_targets=True)
            elif command in ["go_wp", "strafe_wp"] and len(cmd) > 1:
                self._handle_move_command(command.replace("_wp", ""), cmd[1], use_stare_targets=False)
            elif command in ["climb", "descend"] and len(cmd) > 1:
                self._handle_altitude_change_command(command, cmd[1])
            elif command == "maintain" and len(cmd) > 1:
                self._handle_maintain_command(cmd[1])
            elif command == "moveto" and len(cmd) > 3:
                self._handle_moveto_command(cmd[1:])
            elif command == "head" and len(cmd) > 1:
                self._handle_head_command(cmd[1])
            elif command == "look" and len(cmd) > 1:
                self._handle_look_command(cmd[1])
            elif command == "stare" and len(cmd) > 1:
                self._handle_stare_command(cmd[1])
            elif command == "gimbal" and len(cmd) > 1:
                self._handle_gimbal_command(cmd[1:])
            else:
                self.get_logger().warn(f"알 수 없는 명령: '{line.strip()}'")
    
    def _handle_start_command(self):
        if self.state == "INIT":
            self.get_logger().info("사용자 명령: START. 드론 시동 및 ARM.")
            self.start_mission()
        else:
            self.get_logger().warn(f"START 명령을 사용할 수 없는 상태: {self.state}")
    
    def _handle_takeoff_command(self):
        if self.state == "ARMED_IDLE":
            self.state = "TAKING_OFF"
        else:
            self.get_logger().warn(f"이륙할 수 없는 상태: {self.state}")
    
    def _handle_land_command(self):
        if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
            self.state = "LANDING"
        else:
            self.get_logger().warn(f"착륙할 수 없는 상태: {self.state}")
    
    def _handle_arm_command(self):
        if self.state == "LANDED":
            self.get_logger().info("사용자 명령: ARM. 드론 재시동.")
            dcu.arm_and_offboard(self)
            self.state = "ARMED_IDLE"
        else:
            self.get_logger().warn(f"ARM은 LANDED 상태에서만 가능. 현재 상태: {self.state}")
    
    def _handle_stop_command(self):
        if self.state in ["IDLE", "MOVING", "TAKING_OFF", "HEADING"]:
            if self.current_map_pose:
                self.get_logger().info("사용자 명령: STOP. 이동 정지.")
                self.target_pose_map = copy.deepcopy(self.current_map_pose)
                self.target_yaw_deg = None  # yaw 제어 해제
                self.state = "IDLE"
            else:
                self.get_logger().warn("현재 위치를 알 수 없어 정지할 수 없음")
        else:
            self.get_logger().warn(f"정지할 수 없는 상태: {self.state}")
    
    def _handle_move_command(self, command, target_str, use_stare_targets=True):
        try:
            if self.state not in ["IDLE", "MOVING", "HEADING"]:
                self.get_logger().warn(f"'{command}' 명령을 실행할 수 없는 상태: {self.state}")
                return
            
            if target_str == "final":
                if use_stare_targets:
                    wp = self.final_destination
                    target_yaw = None  # 주시 타겟으로 이동 시에는 yaw 제어 안함
                    self.get_logger().info(f"사용자 명령: {command.upper()} to final destination (stare target).")
                else:
                    # 드론 웨이포인트에서는 마지막 웨이포인트 사용
                    final_wp_index = len(self.drone_waypoints) - 1
                    wp = self.drone_waypoints[final_wp_index].tolist()
                    target_yaw = self.get_waypoint_yaw(final_wp_index)
                    self.get_logger().info(f"사용자 명령: {command.upper()} to final waypoint (drone waypoint, yaw: {target_yaw:.0f}도).")
            else:
                wp_index = int(target_str)
                if use_stare_targets:
                    if wp_index >= len(self.stare_targets):
                        self.get_logger().error(f"주시 타겟 인덱스 {wp_index}가 범위를 벗어났습니다. (0-{len(self.stare_targets)-1})")
                        return
                    wp = self.stare_targets[wp_index]
                    self.get_logger().info(f"사용자 명령: {command.upper()} to stare target {wp_index}.")
                    # 주시 타겟으로 이동 시에는 yaw 제어 안함
                    target_yaw = None
                else:
                    if wp_index >= len(self.drone_waypoints):
                        self.get_logger().error(f"드론 웨이포인트 인덱스 {wp_index}가 범위를 벗어났습니다. (0-{len(self.drone_waypoints)-1})")
                        return
                    wp = self.drone_waypoints[wp_index].tolist()
                    target_yaw = self.get_waypoint_yaw(wp_index)  # 웨이포인트의 yaw 값 가져오기
                    self.get_logger().info(f"사용자 명령: {command.upper()} to drone waypoint {wp_index} (yaw: {target_yaw:.0f}도).")
            
            self.target_pose_map.pose.position.x = wp[0]
            self.target_pose_map.pose.position.y = wp[1]
            
            if command == "go":
                self.target_pose_map.pose.position.z = wp[2]
            elif command == "strafe":
                self.target_pose_map.pose.position.z = self.current_map_pose.pose.position.z
            
            # 드론 웨이포인트의 경우 yaw 적용, 주시 타겟의 경우 yaw 제어 해제
            self.target_yaw_deg = target_yaw if not use_stare_targets else None
            self.state = "MOVING"
            
        except (ValueError, IndexError):
            self.get_logger().error(f"잘못된 인덱스입니다: {target_str}")
    
    def _handle_altitude_change_command(self, command, value_str):
        try:
            if self.state not in ["IDLE", "MOVING", "HEADING"]:
                self.get_logger().warn(f"고도 변경을 할 수 없는 상태입니다: {self.state}")
                return
            
            alt_change = float(value_str) * (-1 if command == "descend" else 1)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z += alt_change
            
            self.get_logger().info(f"상대 고도 변경 {alt_change:+.1f}m. 새 목표 Z: {self.target_pose_map.pose.position.z:.2f}m")
            self.target_yaw_deg = None  # yaw 제어 해제
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"잘못된 고도 값입니다: {value_str}")
    
    def _handle_maintain_command(self, value_str):
        try:
            if self.state not in ["IDLE", "MOVING", "HEADING"]:
                self.get_logger().warn(f"고도 유지를 할 수 없는 상태입니다: {self.state}")
                return
            
            target_alt = float(value_str)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z = target_alt
            
            self.get_logger().info(f"절대 고도 {target_alt:.2f}m로 유지합니다.")
            self.target_yaw_deg = None  # yaw 제어 해제
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"잘못된 고도 값입니다: {value_str}")
    
    def _handle_moveto_command(self, args):
        try:
            if self.state not in ["IDLE", "MOVING", "HEADING"]:
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
            self.target_yaw_deg = None  # yaw 제어 해제
            self.state = "MOVING"
            
        except ValueError:
            self.get_logger().error(f"moveto에 잘못된 좌표입니다. 숫자 3개를 입력하세요.")
    
    def _handle_head_command(self, angle_str):
        """드론이 특정 각도 방향을 바라보도록 합니다 (현재 위치 유지)."""
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"'head' 명령을 실행할 수 없는 상태입니다: {self.state}")
                return
            
            target_yaw_deg = float(angle_str)
            
            # 현재 위치를 목표 위치로 설정 (위치는 유지, yaw만 변경)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z = self.current_map_pose.pose.position.z
            
            # 목표 yaw 각도 저장 (임시로 target_pose_map에 저장하기 위해 새로운 속성 추가)
            self.target_yaw_deg = target_yaw_deg
            
            self.get_logger().info(f"사용자 명령: HEAD {target_yaw_deg:.0f}도")
            self.state = "HEADING"  # 새로운 상태 추가
            
        except ValueError:
            self.get_logger().error(f"잘못된 각도 값입니다: {angle_str}")
    
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
                if 0 <= target_index < len(self.stare_targets):
                    self.get_logger().info(f"사용자 명령: LOOK {target_index}")
                    self.point_gimbal_at_target(self.stare_targets[target_index])
                else:
                    self.get_logger().error(f"Look 인덱스 {target_index}가 범위를 벗어났습니다. (0-{len(self.stare_targets)-1})")
            except ValueError:
                self.get_logger().error(f"잘못된 look 명령입니다. 'forward', 'down', 또는 인덱스 0-{len(self.stare_targets)-1}을 사용하세요")
    
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
                if 0 <= target_index < len(self.stare_targets):
                    self.get_logger().info(f"사용자 명령: STARE {target_index}. 주시 타겟을 계속 추적합니다.")
                    self.stare_target_index = target_index
                    # 즉시 한번 조준 실행
                    self.point_gimbal_at_target(self.stare_targets[self.stare_target_index])
                else:
                    self.get_logger().error(f"Stare 인덱스 {target_index}가 범위를 벗어났습니다. (0-{len(self.stare_targets)-1})")
            except ValueError:
                self.get_logger().error(f"잘못된 stare 명령입니다. 'stop' 또는 인덱스 0-{len(self.stare_targets)-1}을 사용하세요")
    
    def _handle_gimbal_command(self, args):
        """짐벌 관련 명령을 처리합니다."""
        if not args:
            self.get_logger().error("짐벌 명령이 불완전합니다.")
            return
        
        sub_command = args[0].lower()
        
        if sub_command == "set" and len(args) >= 3:
            self._handle_gimbal_set_command(args[1:])
        else:
            self.get_logger().error("사용법: gimbal set <pitch> <yaw> [roll]")
    
    def _handle_gimbal_set_command(self, args):
        """짐벌 각도 설정 명령을 처리합니다."""
        try:
            if len(args) < 2:
                self.get_logger().error("사용법: gimbal set <pitch> <yaw> [roll]")
                return
            
            pitch = float(args[0])
            yaw = float(args[1])
            roll = float(args[2]) if len(args) > 2 else 0.0
            
            self.get_logger().info(f"짐벌 각도 설정: Pitch={pitch:.0f}°, Yaw={yaw:.0f}°, Roll={roll:.0f}°")
            dcu.set_gimbal_angle(self, pitch_deg=pitch, yaw_deg=yaw, roll_deg=roll)
            
        except ValueError:
            self.get_logger().error("각도 값이 잘못되었습니다. 숫자를 입력하세요.")
    

            # 시각화
    
    def _publish_all_markers(self):
        """모든 웨이포인트와 주시 타겟 위치에 마커를 게시합니다."""
        marker_array = visu.create_interactive_mission_markers(
            self, self.drone_waypoints.tolist(), self.stare_targets, self.final_destination
        )
        self.visual_marker_publisher.publish(marker_array)
    
    # 미션 로직 구현 (BaseMissionNode의 추상 메서드)
    
    def run_mission_logic(self):
        """대화형 미션의 상태 머신 로직을 구현합니다."""
        
        # Stare 모드 실행
        if self.stare_target_index is not None and self.state in ["IDLE", "MOVING"]:
            self.point_gimbal_at_target(self.stare_targets[self.stare_target_index])
        
        # 시각화 마커 퍼블리시
        self._publish_all_markers()
        
        # 미션별 상태 처리
        if self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING":
            self._handle_moving_state()
        elif self.state == "HEADING":
            self._handle_heading_state()
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
    
    def _handle_heading_state(self):
        """방향 회전 상태 처리"""
        target_pos = [
            self.target_pose_map.pose.position.x,
            self.target_pose_map.pose.position.y,
            self.target_pose_map.pose.position.z
        ]
        
        # 위치와 yaw 모두 제어
        self.publish_position_setpoint(target_pos, self.target_yaw_deg)
        
        # 위치 도착 확인 (yaw는 별도로 확인하지 않고 일정 시간 후 완료로 간주)
        if self.check_arrival(target_pos, tolerance=1.0):
            self.get_logger().info(f"방향 회전 완료 ({self.target_yaw_deg:.0f}도). 호버링 상태.")
            self.state = "IDLE"
    
    def _handle_idle_state(self):
        """대기 상태 처리"""
        target_pos = [
            self.target_pose_map.pose.position.x,
            self.target_pose_map.pose.position.y,
            self.target_pose_map.pose.position.z
        ]
        # IDLE 상태에서도 target_yaw_deg가 설정되어 있으면 yaw 제어
        self.publish_position_setpoint(target_pos, self.target_yaw_deg)
    
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