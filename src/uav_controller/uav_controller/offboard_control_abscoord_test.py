#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleLandDetected, VehicleAttitude, GimbalDeviceAttitudeStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# --- TF2 관련 모듈 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import math
import threading
import sys
import copy

class InteractiveMissionNode(Node):
    """
    사용자 입력에 따라 드론을 제어하는 오프보드 제어 노드.
    - 이륙, 착륙, 정지, 재시동, 지점 이동, 고도 및 짐벌 변경, 특정 지점 응시(stare) 기능을 수행합니다.
    """

    def __init__(self):
        super().__init__('interactive_mission_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # --- 퍼블리셔 ---
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.state_publisher = self.create_publisher(String, "/drone/state", 10)

        # --- 서브스크라이버 ---
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile
        )
        self.land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.land_detected_callback, qos_profile
        )
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile
        )
        self.gimbal_status_subscriber = self.create_subscription(
            GimbalDeviceAttitudeStatus, "/fmu/out/gimbal_device_attitude_status", self.gimbal_status_callback, qos_profile
        )

        # --- TF2 리스너 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 미션 변수 ---
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.land_detected = None
        self.current_attitude = None
        self.target_pose_map = PoseStamped()
        self.target_pose_map.header.frame_id = 'map'
        self.takeoff_altitude = 10.0
        self.takeoff_target_local = None 
        
        self.handshake_counter = 0
        self.handshake_duration = 15
        self.drone_frame_id = "x500_gimbal_0/base_link"

        # <<< CHANGE: 'stare' 기능을 위한 상태 변수 추가 >>>
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
        self.final_destination = [-62.9630, 99.0915, 0.1349] # 'final' 목적지
        
        # --- 사용자 입력을 위한 스레드 ---
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("Interactive Mission Controller is running.")
        self.get_logger().info("Waiting for TF and Local Position data to start...")

    def command_input_loop(self):
        """사용자 명령을 터미널에서 입력받는 루프."""
        print("\n--- Drone Command Console ---")
        print("  [비행 제어]")
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
        # <<< CHANGE: 명령어 도움말 업데이트 >>>
        print("  [짐벌 제어]")
        print("    look <0-5>               - 지정 번호의 마커를 한번 바라봄")
        print("    look front               - 짐벌 정면으로 초기화")
        print("    look down                - 짐벌 수직 아래로")
        print("    stare <0-5>              - 지정 번호의 마커를 계속 추적/응시")
        print("    stare stop               - 추적/응시 중지")
        # <<< CHANGE END >>>
        print("-----------------------------")

        for line in sys.stdin:
            cmd = line.strip().split()
            if not cmd: continue
            command = cmd[0].lower()
            
            if command == "takeoff":
                if self.state == "ARMED_IDLE": self.state = "TAKING_OFF"
                else: self.get_logger().warn(f"Cannot takeoff, state: {self.state}")
            elif command == "land":
                if self.state in ["IDLE", "MOVING", "TAKING_OFF"]: self.state = "LANDING"
                else: self.get_logger().warn(f"Cannot land from state: {self.state}")
            elif command == "arm":
                if self.state == "LANDED":
                    self.get_logger().info("User command: ARM. Re-arming drone.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                    self.state = "ARMED_IDLE"
                else: self.get_logger().warn(f"Can only arm when LANDED. State: {self.state}")
            elif command == "stop":
                if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
                    if self.current_map_pose:
                        self.get_logger().info("User command: STOP. Halting movement.")
                        self.target_pose_map = copy.deepcopy(self.current_map_pose)
                        self.state = "IDLE"
                    else: self.get_logger().warn("Cannot stop, current pose not available.")
                else: self.get_logger().warn(f"Cannot stop from state: {self.state}")
            elif command in ["go", "strafe"] and len(cmd) > 1:
                self.handle_move_command(command, cmd[1])
            elif command in ["climb", "descend"] and len(cmd) > 1:
                self.handle_altitude_change_command(command, cmd[1])
            elif command == "maintain" and len(cmd) > 1:
                self.handle_maintain_command(cmd[1])
            # <<< CHANGE: 짐벌 제어 로직 수정 >>>
            elif command == "look" and len(cmd) > 1:
                self.handle_look_command(cmd[1])
            elif command == "stare" and len(cmd) > 1:
                self.handle_stare_command(cmd[1])
            # <<< CHANGE END >>>
            else:
                self.get_logger().warn(f"Unknown command: '{line.strip()}'")

    def handle_move_command(self, command, target_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"Cannot execute '{command}' in state: {self.state}.")
                return
            if target_str == "final":
                wp = self.final_destination
                self.get_logger().info(f"User command: {command.upper()} to final destination.")
            else:
                wp_index = int(target_str)
                wp = self.waypoints[wp_index]
                self.get_logger().info(f"User command: {command.upper()} to waypoint {wp_index}.")
            self.target_pose_map.pose.position.x = wp[0]
            self.target_pose_map.pose.position.y = wp[1]
            if command == "go":
                self.target_pose_map.pose.position.z = wp[2]
            elif command == "strafe":
                self.target_pose_map.pose.position.z = self.current_map_pose.pose.position.z
            self.state = "MOVING"
        except (ValueError, IndexError):
            self.get_logger().error(f"Invalid waypoint index. Use 0-{len(self.waypoints)-1} or 'final'")

    def handle_altitude_change_command(self, command, value_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"Cannot change altitude in state: {self.state}.")
                return
            alt_change = float(value_str) * (-1 if command == "descend" else 1)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z += alt_change
            self.get_logger().info(f"Relative altitude change {alt_change:+.1f}m. New target Z: {self.target_pose_map.pose.position.z:.2f}m")
            self.state = "MOVING"
        except ValueError:
            self.get_logger().error(f"Invalid altitude value: {value_str}")

    def handle_maintain_command(self, value_str):
        try:
            if self.state not in ["IDLE", "MOVING"]:
                self.get_logger().warn(f"Cannot maintain altitude in state: {self.state}.")
                return
            target_alt = float(value_str)
            self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
            self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
            self.target_pose_map.pose.position.z = target_alt
            self.get_logger().info(f"Maintaining absolute altitude at {target_alt:.2f}m.")
            self.state = "MOVING"
        except ValueError:
            self.get_logger().error(f"Invalid altitude value: {value_str}")

    def handle_look_command(self, sub_command):
        self.stare_target_index = None # 'look' 명령은 항상 'stare'를 중지시킴
        sub_command = sub_command.lower()
        if sub_command == "front":
            self.get_logger().info("User command: LOOK FRONT")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=0.0, param3=0.0, param7=2.0)
        elif sub_command == "down":
            self.get_logger().info("User command: LOOK DOWN")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=-90.0, param3=0.0, param7=2.0)
        else:
            try:
                self.point_gimbal_at_target(int(sub_command))
            except (ValueError, IndexError):
                self.get_logger().error(f"Invalid look command. Use 'front', 'down', or index 0-{len(self.waypoints)-1}")

    def handle_stare_command(self, sub_command):
        sub_command = sub_command.lower()
        if sub_command == "stop":
            if self.stare_target_index is not None:
                self.get_logger().info("User command: STARE STOP. Stopping gimbal tracking.")
                self.stare_target_index = None
            else:
                self.get_logger().info("Not currently staring at anything.")
        else:
            try:
                target_index = int(sub_command)
                if 0 <= target_index < len(self.waypoints):
                    self.get_logger().info(f"User command: STARE {target_index}. Continuously tracking waypoint.")
                    self.stare_target_index = target_index
                    # 즉시 한번 조준 실행
                    self.point_gimbal_at_target(self.stare_target_index)
                else:
                    self.get_logger().error(f"Stare index {target_index} out of bounds.")
            except ValueError:
                self.get_logger().error(f"Invalid stare command. Use 'stop' or index 0-{len(self.waypoints)-1}")

    def attitude_callback(self, msg: VehicleAttitude): self.current_attitude = msg
    def land_detected_callback(self, msg: VehicleLandDetected): self.land_detected = msg

    def gimbal_status_callback(self, msg: GimbalDeviceAttitudeStatus):
        q = msg.q
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(
            f"Gimbal Status -> Pitch: {math.degrees(pitch):.1f} deg, Yaw (relative): {math.degrees(yaw):.1f} deg",
            throttle_duration_sec=1
        )

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.current_local_pos = msg
        self.run_state_machine()

    def update_current_map_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            if self.current_map_pose is None: self.current_map_pose = PoseStamped()
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            return True
        except TransformException as e:
            if self.state != "INIT": self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=1.0)
            return False

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=True, timestamp=int(self.get_clock().now().nanoseconds / 1000))
        self.offboard_control_mode_publisher.publish(msg)

    def publish_map_based_setpoint(self):
        if self.current_map_pose is None or self.current_local_pos is None: return
        delta_map_x = self.target_pose_map.pose.position.x - self.current_map_pose.pose.position.x
        delta_map_y = self.target_pose_map.pose.position.y - self.current_map_pose.pose.position.y
        delta_map_z = self.target_pose_map.pose.position.z - self.current_map_pose.pose.position.z
        delta_ned_x, delta_ned_y, delta_ned_z = delta_map_y, delta_map_x, -delta_map_z
        target_ned_x = self.current_local_pos.x + delta_ned_x
        target_ned_y = self.current_local_pos.y + delta_ned_y
        target_ned_z = self.current_local_pos.z + delta_ned_z
        msg = TrajectorySetpoint(position=[float(target_ned_x), float(target_ned_y), float(target_ned_z)], yaw=0.0, timestamp=int(self.get_clock().now().nanoseconds / 1000))
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand(command=command, timestamp=int(self.get_clock().now().nanoseconds / 1000), from_external=True, target_system=1, target_component=1)
        for i in range(1, 8): msg.__setattr__(f'param{i}', float(kwargs.get(f"param{i}", 0.0)))
        self.vehicle_command_publisher.publish(msg)

    def point_gimbal_at_target(self, target_index):
        """
        드론의 현재 ENU 위치와 목표 ENU 위치 사이의 각도를 직접 계산하여 짐벌을 제어합니다.
        
        이 방식은 GPS 좌표 변환 과정을 완전히 생략합니다.
        시뮬레이션의 월드(map) 좌표계와 드론이 사용하는 내부 기준점(ref_lat/lon)이
        일치하지 않을 때 발생하는 문제를 원천적으로 차단하기 때문에 훨씬 안정적이고 정확합니다.
        """
        if self.current_map_pose is None or self.current_attitude is None:
            self.get_logger().warn("Drone pose or attitude not available for gimbal control.")
            return
        if not (0 <= target_index < len(self.waypoints)):
            self.get_logger().error(f"Waypoint index {target_index} out of bounds.")
            return

        drone_pos = self.current_map_pose.pose.position
        target_pos = self.waypoints[target_index]
        
        delta_x = target_pos[0] - drone_pos.x  # East
        delta_y = target_pos[1] - drone_pos.y  # North
        delta_z = target_pos[2] - drone_pos.z  # Up
        
        # Pitch 계산 (수평거리를 기준으로, 목표가 아래에 있으면 음수)
        distance_2d = math.sqrt(delta_x**2 + delta_y**2)
        pitch_rad = math.atan2(delta_z, distance_2d)
        
        # Yaw 계산 (지도 기준 Yaw를 PX4가 사용하는 '북쪽 기준, 시계방향' Yaw로 변환)
        map_yaw_rad = math.atan2(delta_y, delta_x) # ENU 기준 (East=0, North=90)
        map_yaw_deg = math.degrees(map_yaw_rad)
        px4_yaw_deg = 90.0 - map_yaw_deg # PX4 기준 (North=0, East=90)
        if px4_yaw_deg > 180: px4_yaw_deg -= 360.0
        if px4_yaw_deg < -180: px4_yaw_deg += 360.0
        
        # self.get_logger().info(f"Pointing gimbal to ENU target {target_index}. Pitch: {math.degrees(pitch_rad):.1f}, Yaw: {px4_yaw_deg:.1f}", throttle_duration_sec=1)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=math.degrees(pitch_rad), param3=px4_yaw_deg, param7=2.0)
    
    def check_arrival(self, tolerance=1.5):
        if self.current_map_pose is None: return False
        pos = self.current_map_pose.pose.position
        target_pos = self.target_pose_map.pose.position
        return math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2) < tolerance

    def run_state_machine(self):
        if not self.update_current_map_pose() or self.current_local_pos is None or self.land_detected is None:
            return
        
        # <<< CHANGE: 'stare' 모드 실행 로직 추가 >>>
        if self.stare_target_index is not None and self.state in ["IDLE", "MOVING"]:
            self.point_gimbal_at_target(self.stare_target_index)

        state_msg = String(data=self.state)
        self.state_publisher.publish(state_msg)

        if self.state not in ["LANDING", "LANDED", "INIT"]:
             self.publish_offboard_control_mode()

        if self.state == "INIT":
            self.get_logger().info("System ready, waiting for first local position to start handshake.", once=True)
            if self.current_local_pos: self.state = "HANDSHAKE"
        elif self.state == "HANDSHAKE":
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Handshake complete. Switching to Offboard mode and Arming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info("Arm command sent. Ready for 'takeoff' command.")
                self.state = "ARMED_IDLE"
        elif self.state == "ARMED_IDLE":
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
        elif self.state == "TAKING_OFF":
            if self.takeoff_target_local is None:
                self.get_logger().info(f"Takeoff initiated. Target altitude: {self.takeoff_altitude}m.")
                self.takeoff_target_local = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z - self.takeoff_altitude]
            sp_msg = TrajectorySetpoint(position=[float(p) for p in self.takeoff_target_local], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 1.0:
                self.get_logger().info("Takeoff complete. Hovering.")
                self.target_pose_map = copy.deepcopy(self.current_map_pose)
                self.state = "IDLE"
                self.takeoff_target_local = None
        elif self.state == "MOVING":
            self.publish_map_based_setpoint()
            if self.check_arrival():
                self.get_logger().info("Destination reached. Hovering.")
                self.state = "IDLE"
        elif self.state == "IDLE":
            self.publish_map_based_setpoint()
        elif self.state == "LANDING":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.land_detected.landed:
                self.get_logger().info("Landed successfully. Ready for 'arm' command.")
                self.state = "LANDED"
        elif self.state == "LANDED":
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = InteractiveMissionNode()
    try:
        rclpy.spin(mission_node)
    except (KeyboardInterrupt, SystemExit):
        mission_node.get_logger().info("Shutdown requested. Forcing landing.")
        if hasattr(mission_node, 'state') and mission_node.state not in ["LANDED", "INIT"]:
             mission_node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    finally:
        if rclpy.ok():
            mission_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
