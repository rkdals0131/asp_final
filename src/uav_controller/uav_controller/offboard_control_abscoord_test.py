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

class InteractiveMissionNode(Node):
    """
    사용자 입력에 따라 드론을 제어하는 오프보드 제어 노드.
    - 별도 스레드에서 사용자 명령을 받아 처리합니다.
    - 이륙, 착륙, 특정 지점 이동, 고도 변경 등의 기능을 수행합니다.
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
        print("명령어 예시:")
        print("  takeoff                  - 이륙")
        print("  land                     - 착륙")
        print("  go <0-5>                 - N번 웨이포인트로 이동")
        print("  go final                 - 최종 목적지로 이동")
        print("  up <meters>              - 지정한 미터만큼 고도 상승")
        print("  down <meters>            - 지정한 미터만큼 고도 하강")
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
                    self.get_logger().warn("Takeoff is only possible from ARMED_IDLE state.")

            elif command == "land":
                self.get_logger().info("User command: LAND")
                self.state = "LANDING"

            elif command == "go" and len(cmd) > 1:
                target = cmd[1].lower()
                try:
                    if self.state not in ["IDLE", "MOVING"]:
                        self.get_logger().warn(f"Cannot execute 'go' in state: {self.state}. Must be flying (IDLE).")
                        continue

                    if target == "final":
                        wp = self.final_destination
                        self.get_logger().info(f"User command: GO to final destination {wp}")
                    else:
                        wp_index = int(target)
                        if 0 <= wp_index < len(self.waypoints):
                            wp = self.waypoints[wp_index]
                            self.get_logger().info(f"User command: GO to waypoint {wp_index} {wp}")
                        else:
                            self.get_logger().warn(f"Invalid waypoint index: {wp_index}. Available: 0-{len(self.waypoints)-1}")
                            continue
                    
                    self.target_pose_map.pose.position.x = wp[0]
                    self.target_pose_map.pose.position.y = wp[1]
                    self.target_pose_map.pose.position.z = wp[2]
                    self.state = "MOVING"

                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"Invalid 'go' command: {e}")

            elif command in ["up", "down"] and len(cmd) > 1:
                try:
                    if self.state not in ["IDLE", "MOVING"]:
                        self.get_logger().warn(f"Cannot change altitude in state: {self.state}. Must be flying (IDLE).")
                        continue

                    alt_change = float(cmd[1])
                    if command == "down":
                        alt_change *= -1
                    
                    # 현재 맵 좌표 기준으로 고도 변경
                    self.target_pose_map.pose.position.x = self.current_map_pose.pose.position.x
                    self.target_pose_map.pose.position.y = self.current_map_pose.pose.position.y
                    self.target_pose_map.pose.position.z += alt_change
                    self.get_logger().info(f"User command: Altitude change {alt_change:+.1f}m. New target Z: {self.target_pose_map.pose.position.z:.2f}m")
                    self.state = "MOVING"

                except ValueError as e:
                    self.get_logger().error(f"Invalid altitude command: {e}")
            else:
                self.get_logger().warn(f"Unknown command: '{line.strip()}'")


    def local_position_callback(self, msg: VehicleLocalPosition):
        """VehicleLocalPosition 콜백, 현재 로컬 위치를 업데이트하고 상태 머신을 실행합니다."""
        self.current_local_pos = msg
        # 상태 머신을 주기적으로 직접 실행하도록 타이머로 변경하는 것을 고려할 수 있습니다.
        # 여기서는 콜백 기반으로 유지합니다.
        self.run_state_machine()

    def update_current_map_pose(self):
        """TF 리스너를 사용하여 'map' 프레임에서 드론의 현재 절대 위치를 업데이트합니다."""
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
        """목표 map 포즈를 기반으로 TrajectorySetpoint을 계산하고 퍼블리시합니다."""
        if self.current_map_pose is None or self.current_local_pos is None: return

        delta_map_x = self.target_pose_map.pose.position.x - self.current_map_pose.pose.position.x
        delta_map_y = self.target_pose_map.pose.position.y - self.current_map_pose.pose.position.y
        delta_map_z = self.target_pose_map.pose.position.z - self.current_map_pose.pose.position.z
        
        # map 프레임(ENU/FLU) 델타를 드론의 로컬 NED 프레임 델타로 변환
        # map_x(East) -> ned_y(East), map_y(North) -> ned_x(North), map_z(Up) -> -ned_z(Down)
        delta_ned_x = delta_map_y
        delta_ned_y = delta_map_x
        delta_ned_z = -delta_map_z

        target_ned_x = self.current_local_pos.x + delta_ned_x
        target_ned_y = self.current_local_pos.y + delta_ned_y
        target_ned_z = self.current_local_pos.z + delta_ned_z

        msg = TrajectorySetpoint()
        msg.position = [float(target_ned_x), float(target_ned_y), float(target_ned_z)]
        msg.yaw = 0.0 # 북쪽을 계속 바라보도록 설정
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand(command=command, timestamp=int(self.get_clock().now().nanoseconds / 1000))
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def check_arrival(self, tolerance=0.8):
        """드론이 목표 map 위치에 도착했는지 확인합니다."""
        if self.current_map_pose is None: return False
        pos = self.current_map_pose.pose.position
        target_pos = self.target_pose_map.pose.position
        distance = math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2)
        return distance < tolerance

    def run_state_machine(self):
        """메인 상태 머신, local_position_callback에 의해 실행됩니다."""
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return

        state_msg = String(data=self.state)
        self.state_publisher.publish(state_msg)

        if self.state not in ["LANDING", "MISSION_COMPLETE", "INIT"]:
             self.publish_offboard_control_mode()

        # --- 상태 로직 ---
        if self.state == "INIT":
            # TF와 Local Position이 수신될 때까지 대기
            self.get_logger().info("TF and Local Position received. Starting handshake.", once=True)
            self.state = "HANDSHAKE"

        elif self.state == "HANDSHAKE":
            # 오프보드 모드 연결을 위해 현재 위치를 setpoint로 계속 전송
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Handshake complete. Switching to Offboard mode and Arming.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # 6 = Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # 1 = Arm
                self.get_logger().info("Arm command sent. Drone is in ARMED_IDLE state. Ready for 'takeoff' command.")
                self.state = "ARMED_IDLE"

        elif self.state == "ARMED_IDLE":
            # 시동 걸린 채로 지상에서 대기. 사용자 명령을 기다림.
            # 안정성을 위해 현재 위치 유지를 위한 setpoint 전송
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
        elif self.state == "TAKING_OFF":
            # 로컬 좌표 기준으로 안전하게 이륙
            takeoff_target_local = [self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z - self.takeoff_altitude]
            sp_msg = TrajectorySetpoint(position=[float(p) for p in takeoff_target_local], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)
            
            # 목표 고도의 95% 이상 도달했는지 확인 (local Z는 음수이므로 부등호 주의)
            if self.current_local_pos.z < takeoff_target_local[2] + self.takeoff_altitude * 0.05:
                self.get_logger().info("Takeoff complete. Hovering and waiting for command.")
                self.target_pose_map = self.current_map_pose # 현재 위치를 목표로 설정하여 안정적인 호버링
                self.state = "IDLE"
        
        elif self.state == "MOVING":
            self.publish_map_based_setpoint()
            if self.check_arrival():
                self.get_logger().info("Destination reached. Hovering.")
                self.state = "IDLE"

        elif self.state == "IDLE":
            # 목표 지점에 도달 후 공중에서 호버링. 새로운 명령을 기다림.
            self.publish_map_based_setpoint()

        elif self.state == "LANDING":
            self.get_logger().info("Landing command sent.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = "MISSION_COMPLETE"
        
        elif self.state == "MISSION_COMPLETE":
            # 아무것도 하지 않음. PX4가 자동으로 disarm 함.
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = InteractiveMissionNode()
    try:
        rclpy.spin(mission_node)
    except (KeyboardInterrupt, SystemExit):
        mission_node.get_logger().info("Shutdown requested. Forcing landing.")
        # 비상 상황시 land 명령 전송
        mission_node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    finally:
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
