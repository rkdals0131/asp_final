#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
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
    - 이륙, 착륙, 정지, 재시동, 지점 이동, 고도 및 짐벌 변경 등의 기능을 수행합니다.
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

        # --- TF2 리스너 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 미션 변수 ---
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.target_pose_map = PoseStamped()
        self.target_pose_map.header.frame_id = 'map'
        self.takeoff_altitude = 10.0
        self.takeoff_target_local = None 
        
        self.handshake_counter = 0
        self.handshake_duration = 15
        self.drone_frame_id = "x500_gimbal_0/base_link"

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
        print("  [짐벌 제어]")
        print("    gimbal_angle <pitch> [yaw] - 짐벌 각도 조작 (예: gimbal_angle -90 0)")
        print("    gimbal_reset             - 짐벌 정면으로 초기화")
        print("    gimbal_down              - 짐벌 수직 아래로")
        print("-----------------------------")

        for line in sys.stdin:
            cmd = line.strip().split()
            if not cmd:
                continue

            command = cmd[0].lower()
            
            # --- 명령어 처리 로직 ---
            if command == "takeoff":
                if self.state == "ARMED_IDLE":
                    self.get_logger().info("User command: TAKEOFF")
                    self.state = "TAKING_OFF"
                else:
                    self.get_logger().warn(f"Takeoff is only possible from ARMED_IDLE state. Current state: {self.state}")

            elif command == "land":
                if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
                    self.get_logger().info("User command: LAND")
                    self.state = "LANDING"
                else:
                    self.get_logger().warn(f"Cannot land from state: {self.state}")

            elif command == "arm":
                if self.state == "LANDED":
                    self.get_logger().info("User command: ARM. Re-arming the drone.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # Offboard 모드 재설정
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # Arm
                    self.state = "ARMED_IDLE"
                else:
                     self.get_logger().warn(f"Can only arm when LANDED. Current state: {self.state}")

            elif command == "stop":
                if self.state in ["IDLE", "MOVING", "TAKING_OFF"]:
                    self.get_logger().info("User command: STOP. Halting movement and hovering.")
                    self.target_pose_map = copy.deepcopy(self.current_map_pose) # 현재 위치를 목표로 설정
                    self.state = "IDLE"
                else:
                    self.get_logger().warn(f"Cannot stop from state: {self.state}")

            elif command in ["go", "strafe"] and len(cmd) > 1:
                target = cmd[1].lower()
                try:
                    if self.state not in ["IDLE", "MOVING"]:
                        self.get_logger().warn(f"Cannot execute '{command}' in state: {self.state}. Must be flying (IDLE or MOVING).")
                        continue

                    if target == "final":
                        wp = self.final_destination
                        self.get_logger().info(f"User command: {command.upper()} to final destination.")
                    else:
                        wp_index = int(target)
                        wp = self.waypoints[wp_index]
                        self.get_logger().info(f"User command: {command.upper()} to waypoint {wp_index}.")
                    
                    self.target_pose_map.pose.position.x = wp[0]
                    self.target_pose_map.pose.position.y = wp[1]
                    
                    if command == "go":
                        self.target_pose_map.pose.position.z = wp[2]
                    elif command == "strafe":
                        current_alt = self.current_map_pose.pose.position.z
                        self.get_logger().info(f"Maintaining current altitude of {current_alt:.2f}m")
                        self.target_pose_map.pose.position.z = current_alt
                    
                    self.state = "MOVING"

                except (ValueError, IndexError):
                    self.get_logger().error(f"Invalid '{command}' command. Waypoint index must be 0-{len(self.waypoints)-1} or 'final'")

            elif command in ["climb", "descend"] and len(cmd) > 1:
                try:
                    if self.state not in ["IDLE", "MOVING"]:
                        self.get_logger().warn(f"Cannot change altitude in state: {self.state}. Must be flying (IDLE or MOVING).")
                        continue

                    alt_change = float(cmd[1])
                    if command == "descend":
                        alt_change *= -1
                    
                    self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
                    self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
                    self.target_pose_map.pose.position.z += alt_change
                    self.get_logger().info(f"User command: Relative altitude change {alt_change:+.1f}m. New target Z: {self.target_pose_map.pose.position.z:.2f}m")
                    self.state = "MOVING"

                except ValueError:
                    self.get_logger().error(f"Invalid altitude command: {cmd[1]} is not a number.")

            elif command == "maintain" and len(cmd) > 1:
                try:
                    if self.state not in ["IDLE", "MOVING"]:
                        self.get_logger().warn(f"Cannot maintain altitude in state: {self.state}. Must be flying (IDLE or MOVING).")
                        continue

                    target_alt = float(cmd[1])
                    self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
                    self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
                    self.target_pose_map.pose.position.z = target_alt
                    self.get_logger().info(f"User command: Maintaining absolute altitude at {target_alt:.2f}m.")
                    self.state = "MOVING"

                except ValueError:
                    self.get_logger().error(f"Invalid altitude for maintain: {cmd[1]} is not a number.")

            # --- 짐벌 명령어 처리 ---
            elif command == "gimbal_angle" and len(cmd) >= 2:
                try:
                    pitch = float(cmd[1])
                    yaw = float(cmd[2]) if len(cmd) >= 3 else 0.0
                    self.get_logger().info(f"User command: GIMBAL_ANGLE to Pitch: {pitch}, Yaw: {yaw}")
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
                        param1=pitch, param2=0.0, param3=yaw, param7=2.0
                    )
                except ValueError:
                    self.get_logger().error(f"Invalid angle value in gimbal command.")

            elif command == "gimbal_reset":
                self.get_logger().info("User command: GIMBAL_RESET (pointing forward)")
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
                    param1=0.0, param2=0.0, param3=0.0, param7=2.0
                )

            elif command == "gimbal_down":
                self.get_logger().info("User command: GIMBAL_DOWN (pointing down)")
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
                    param1=-90.0, param2=0.0, param3=0.0, param7=2.0
                )

            else:
                self.get_logger().warn(f"Unknown command: '{line.strip()}'")

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.current_local_pos = msg
        self.run_state_machine()

    def update_current_map_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            if self.current_map_pose is None:
                self.current_map_pose = PoseStamped()
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            return True
        except TransformException as e:
            if self.state != "INIT":
                self.get_logger().warn(f"Could not get current drone pose in map frame: {e}", throttle_duration_sec=1.0)
            return False

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=True, timestamp=int(self.get_clock().now().nanoseconds / 1000))
        self.offboard_control_mode_publisher.publish(msg)

    def publish_map_based_setpoint(self):
        if self.current_map_pose is None or self.current_local_pos is None: return

        delta_map_x = self.target_pose_map.pose.position.x - self.current_map_pose.pose.position.x
        delta_map_y = self.target_pose_map.pose.position.y - self.current_map_pose.pose.position.y
        delta_map_z = self.target_pose_map.pose.position.z - self.current_map_pose.pose.position.z
        
        delta_ned_x = delta_map_y
        delta_ned_y = delta_map_x
        delta_ned_z = -delta_map_z

        target_ned_x = self.current_local_pos.x + delta_ned_x
        target_ned_y = self.current_local_pos.y + delta_ned_y
        target_ned_z = self.current_local_pos.z + delta_ned_z

        msg = TrajectorySetpoint()
        msg.position = [float(target_ned_x), float(target_ned_y), float(target_ned_z)]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **kwargs):
        """범용 VehicleCommand 메시지 발행 함수"""
        msg = VehicleCommand(command=command, timestamp=int(self.get_clock().now().nanoseconds / 1000))
        msg.param1 = float(kwargs.get("param1", 0.0))
        msg.param2 = float(kwargs.get("param2", 0.0))
        msg.param3 = float(kwargs.get("param3", 0.0))
        msg.param4 = float(kwargs.get("param4", 0.0))
        msg.param5 = float(kwargs.get("param5", 0.0))
        msg.param6 = float(kwargs.get("param6", 0.0))
        msg.param7 = float(kwargs.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1 # 짐벌의 경우 154로 직접 지정할 수도 있음
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def check_arrival(self, tolerance=0.8):
        if self.current_map_pose is None: return False
        pos = self.current_map_pose.pose.position
        target_pos = self.target_pose_map.pose.position
        distance = math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2)
        return distance < tolerance

    def run_state_machine(self):
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return

        state_msg = String(data=self.state)
        self.state_publisher.publish(state_msg)

        if self.state not in ["LANDING", "LANDED", "INIT"]:
             self.publish_offboard_control_mode()

        # --- 상태 로직 ---
        if self.state == "INIT":
            self.get_logger().info("TF and Local Position received. Starting handshake.", once=True)
            self.state = "HANDSHAKE"

        elif self.state == "HANDSHAKE":
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Handshake complete. Switching to Offboard mode and Arming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info("Arm command sent. Drone is in ARMED_IDLE state. Ready for 'takeoff' command.")
                self.state = "ARMED_IDLE"

        elif self.state == "ARMED_IDLE":
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
        elif self.state == "TAKING_OFF":
            if self.takeoff_target_local is None:
                self.get_logger().info(f"Takeoff initiated. Target altitude: {self.takeoff_altitude}m above current position.")
                self.takeoff_target_local = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z - self.takeoff_altitude]

            sp_msg = TrajectorySetpoint(position=[float(p) for p in self.takeoff_target_local], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 1.0:
                self.get_logger().info("Takeoff complete. Hovering and waiting for command.")
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
            # PX4가 착륙을 감지할 때까지 대기
            if self.current_local_pos.landed:
                self.get_logger().info("Landed successfully. Ready to be armed again with 'arm' command.")
                self.state = "LANDED"
        
        elif self.state == "LANDED":
            # 착륙 후 대기 상태. 사용자 'arm' 명령을 기다림.
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = InteractiveMissionNode()
    try:
        rclpy.spin(mission_node)
    except (KeyboardInterrupt, SystemExit):
        mission_node.get_logger().info("Shutdown requested. Forcing landing.")
        mission_node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    finally:
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
