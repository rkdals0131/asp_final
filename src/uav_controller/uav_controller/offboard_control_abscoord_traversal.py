#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

import numpy as np
import math
import threading
import sys
import copy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from mission_admin_interfaces.srv import MissionComplete

# --- TF2 관련 모듈 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class WaypointMissionNode(Node):
    """
    웨이포인트 기반 미션을 수행하는 노드.
    지정된 웨이포인트로 이동하며, 각 지점에서 Stare 타겟을 응시하고 2초간 호버링합니다.
    """
    def __init__(self):
        super().__init__('waypoint_mission_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # --- QOS 프로파일 설정 ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        visual_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=30 # 여러 종류의 마커를 한번에 보내기 위해 깊이 증가
        )

        # --- 퍼블리셔/서브스크라이버 설정 ---
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.state_publisher = self.create_publisher(String, "/drone/state", 10)
        self.visual_marker_publisher = self.create_publisher(MarkerArray, "/mission_visuals", visual_qos_profile)

        self.local_position_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile)
        self.attitude_subscriber = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile)
        self.mission_command_sub = self.create_subscription(String, "/drone/mission_command", self.mission_command_callback, 10)
        
        # 미션 컨트롤 서비스 클라이언트
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')

        # --- TF 설정 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        
        # --- 상태 변수 ---
        self.state = "INIT"
        self.current_map_pose = None
        self.current_local_pos = None
        self.current_attitude = None
        self.drone_frame_id = "x500_gimbal_0"
        self.gimbal_camera_frame_id = "x500_gimbal_0/camera_link"
        self.handshake_counter = 0
        self.handshake_duration = 15
        
        # --- 웨이포인트 미션 관련 변수 ---
        self.current_waypoint_index = 0
        self.hover_start_time = None

        # --- 메인 루프 및 커맨드 입력 ---
        self.state_machine_timer = self.create_timer(0.1, self.run_state_machine) # 10Hz
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("Waypoint Mission Controller is initialized and ready.")

    def command_input_loop(self):
        print("\n--- Waypoint Mission Command ---")
        print("  start   - ARM 후 미션 시작")
        print("  land    - 강제 착륙")
        print("--------------------------------")
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == "start":
                if self.state == "INIT":
                    self.get_logger().info("사용자 명령: START. ARM 후 이륙 시작.")
                    self.state = "HANDSHAKE"
                else:
                    self.get_logger().warn(f"START 명령을 사용할 수 없는 상태입니다: {self.state}")
            elif cmd == "land":
                 if self.state not in ["LANDING", "LANDED"]:
                     self.get_logger().warn("사용자 명령: LAND. 강제 착륙.")
                     self.state = "LANDING"

    def mission_command_callback(self, msg: String):
        """미션 컨트롤 대시보드로부터 명령 수신"""
        command = msg.data.lower()
        if command == 'start':
            if self.state == "INIT":
                self.get_logger().info("🚁 미션 컨트롤로부터 START 명령 수신. ARM 및 이륙 시작")
                self.state = "HANDSHAKE"
            else:
                self.get_logger().warn(f"START 명령을 받았지만 현재 상태가 {self.state}입니다. INIT 상태에서만 시작 가능합니다.")
        elif command == 'land':
            if self.state not in ["LANDING", "LANDED"]:
                self.get_logger().info("⛔ 미션 컨트롤로부터 LAND 명령 수신")
                self.state = "LANDING"

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

    def local_position_callback(self, msg: VehicleLocalPosition): self.current_local_pos = msg
    def attitude_callback(self, msg: VehicleAttitude): self.current_attitude = msg

    def update_current_map_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            if self.current_map_pose is None: self.current_map_pose = PoseStamped()
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            return True
        except TransformException as e:
            if self.state != "INIT": self.get_logger().warn(f"TF lookup failed for '{self.drone_frame_id}': {e}", throttle_duration_sec=1.0)
            return False

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(
            position=True, velocity=False, acceleration=False, attitude=False,
            timestamp=int(self.get_clock().now().nanoseconds / 1000)
        )
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand(command=command, timestamp=int(self.get_clock().now().nanoseconds / 1000), from_external=True, target_system=1, target_component=1)
        for i in range(1, 8): msg.__setattr__(f'param{i}', float(kwargs.get(f"param{i}", 0.0)))
        self.vehicle_command_publisher.publish(msg)

    def publish_position_setpoint(self, target_map_pos):
        if self.current_map_pose is None or self.current_local_pos is None: return
        
        delta_map_x = target_map_pos[0] - self.current_map_pose.pose.position.x
        delta_map_y = target_map_pos[1] - self.current_map_pose.pose.position.y
        delta_map_z = target_map_pos[2] - self.current_map_pose.pose.position.z
        
        target_ned_x = self.current_local_pos.x + delta_map_y
        target_ned_y = self.current_local_pos.y + delta_map_x
        target_ned_z = self.current_local_pos.z - delta_map_z
        
        sp_msg = TrajectorySetpoint(
            position=[target_ned_x, target_ned_y, target_ned_z],
            yaw=math.nan,
            timestamp=int(self.get_clock().now().nanoseconds / 1000)
        )
        self.trajectory_setpoint_publisher.publish(sp_msg)
    
    def point_gimbal_at_target(self, target_enu_pos):
        if self.current_map_pose is None:
            self.get_logger().warn("Drone pose not available for gimbal control.", throttle_duration_sec=2.0)
            return

        drone_pos = self.current_map_pose.pose.position
        target_pos = target_enu_pos
        
        delta_x = target_pos[0] - drone_pos.x
        delta_y = target_pos[1] - drone_pos.y
        delta_z = target_pos[2] - drone_pos.z
        
        distance_2d = math.sqrt(delta_x**2 + delta_y**2)
        pitch_rad = math.atan2(delta_z, distance_2d)
        map_yaw_rad = math.atan2(delta_y, delta_x)
        map_yaw_deg = math.degrees(map_yaw_rad)
        px4_yaw_deg = 90.0 - map_yaw_deg
        
        if px4_yaw_deg > 180.0: px4_yaw_deg -= 360.0
        if px4_yaw_deg < -180.0: px4_yaw_deg += 360.0
        
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
            param1=math.degrees(pitch_rad),
            param3=px4_yaw_deg,
            param7=2.0
        )

    def publish_mission_visuals(self):
        """RViz 시각화를 위한 모든 마커(경로, 타겟, 짐벌 방향)를 생성하고 게시합니다."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # 1. 웨이포인트 경로 마커
        path_marker = Marker()
        path_marker.header.frame_id = "map"; path_marker.header.stamp = now
        path_marker.ns = "waypoint_path"; path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP; path_marker.action = Marker.ADD
        path_marker.scale.x = 0.3
        path_marker.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0)
        path_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
        for wp in self.drone_waypoints:
            path_marker.points.append(Point(x=wp[0], y=wp[1], z=wp[2]))
        marker_array.markers.append(path_marker)

        # 2. 웨이포인트 마커
        for i, wp in enumerate(self.drone_waypoints):
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"; wp_marker.header.stamp = now
            wp_marker.ns = "waypoints"; wp_marker.id = i
            wp_marker.type = Marker.SPHERE; wp_marker.action = Marker.ADD
            wp_marker.pose.position = Point(x=wp[0], y=wp[1], z=wp[2])
            wp_marker.pose.orientation.w = 1.0
            wp_marker.scale.x = wp_marker.scale.y = wp_marker.scale.z = 1.5
            wp_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9) if i == self.current_waypoint_index else ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.7)
            wp_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
            marker_array.markers.append(wp_marker)

        # 3. Stare 타겟 마커 (test.py 스타일)
        colors = [
            (1.0, 0.0, 0.0), (1.0, 0.5, 0.0), (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.5, 0.0, 1.0), (0.8, 0.2, 0.8)
        ]
        for i, target in enumerate(self.stare_targets):
            # 타원 마커
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"; sphere_marker.header.stamp = now
            sphere_marker.ns = "stare_target_spheres"; sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE; sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = Point(x=target[0], y=target[1], z=target[2])
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 3.0; sphere_marker.scale.y = 3.0; sphere_marker.scale.z = 1.0
            r, g, b = colors[i % len(colors)]
            sphere_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.7)
            sphere_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
            marker_array.markers.append(sphere_marker)

            # 실린더 마커
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "map"; cylinder_marker.header.stamp = now
            cylinder_marker.ns = "stare_target_cylinders"; cylinder_marker.id = i
            cylinder_marker.type = Marker.CYLINDER; cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position = Point(x=target[0], y=target[1], z=target[2] + 1.5)
            cylinder_marker.pose.orientation.w = 1.0
            cylinder_marker.scale.x = 0.5; cylinder_marker.scale.y = 0.5; cylinder_marker.scale.z = 3.0
            cylinder_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.7)
            cylinder_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
            marker_array.markers.append(cylinder_marker)

            # 텍스트 마커
            text_marker = Marker()
            text_marker.header.frame_id = "map"; text_marker.header.stamp = now
            text_marker.ns = "stare_target_texts"; text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING; text_marker.action = Marker.ADD
            text_marker.pose.position = Point(x=target[0], y=target[1], z=target[2] - 1.5)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 1.0
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"Target {i}"
            text_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
            marker_array.markers.append(text_marker)

        # 4. 짐벌 방향 화살표 마커
        if self.state != "INIT": # TF가 준비되기 전에는 발행하지 않음
            gimbal_arrow = Marker()
            gimbal_arrow.header.frame_id = self.gimbal_camera_frame_id
            gimbal_arrow.header.stamp = now
            gimbal_arrow.ns = "gimbal_direction_arrow"; gimbal_arrow.id = 0
            gimbal_arrow.type = Marker.ARROW; gimbal_arrow.action = Marker.ADD
            gimbal_arrow.points.append(Point(x=0.0, y=0.0, z=0.0))
            gimbal_arrow.points.append(Point(x=-3.0, y=0.0, z=0.0))
            gimbal_arrow.scale.x = 0.2; gimbal_arrow.scale.y = 0.4
            gimbal_arrow.color = ColorRGBA(r=0.9, g=0.1, b=0.9, a=1.0)
            gimbal_arrow.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(gimbal_arrow)

        self.visual_marker_publisher.publish(marker_array)

    def check_arrival(self, target_pos, tolerance=2.0):
        if self.current_map_pose is None: return False
        pos = self.current_map_pose.pose.position
        return math.sqrt((pos.x - target_pos[0])**2 + (pos.y - target_pos[1])**2 + (pos.z - target_pos[2])**2) < tolerance

    def run_state_machine(self):
        if not self.update_current_map_pose() or self.current_local_pos is None:
            if self.state == "INIT": # 초기화 중에도 마커는 발행
                self.publish_mission_visuals()
            return

        self.publish_mission_visuals()
        self.state_publisher.publish(String(data=self.state))

        if self.state not in ["LANDING", "LANDED", "INIT"]:
             self.publish_offboard_control_mode()

        if self.state == "INIT":
            self.get_logger().info("✅ 시스템 준비 완료. START 명령을 기다리는 중...", once=True)
            # 더 이상 자동으로 HANDSHAKE로 넘어가지 않음

        elif self.state == "HANDSHAKE":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("🔧 ARM 명령 전송 완료. 이륙 시작.")
                self.state = "TAKING_OFF"
        
        elif self.state == "TAKING_OFF":
            if self.current_map_pose:
                takeoff_altitude = self.drone_waypoints[0][2]
                target_pos = [self.current_map_pose.pose.position.x, self.current_map_pose.pose.position.y, takeoff_altitude]
                self.publish_position_setpoint(target_pos)
            
                if abs(self.current_map_pose.pose.position.z - takeoff_altitude) < 1.0:
                    self.get_logger().info(f"🚁 이륙 완료. 첫 번째 웨이포인트 {self.current_waypoint_index}로 이동.")
                    # 미션 컨트롤에 이륙 완료 신호 전송
                    self.send_mission_complete(2)  # DRONE_TAKEOFF_COMPLETE
                    self.state = "MOVING_TO_WAYPOINT"

        elif self.state == "MOVING_TO_WAYPOINT":
            if self.current_waypoint_index >= len(self.drone_waypoints):
                self.state = "LANDING"
                return

            target_wp = self.drone_waypoints[self.current_waypoint_index]
            target_stare_idx = self.stare_indices[self.current_waypoint_index]
            target_stare_pos = self.stare_targets[target_stare_idx]
            
            self.publish_position_setpoint(target_wp)
            self.point_gimbal_at_target(target_stare_pos)

            if self.check_arrival(target_wp):
                self.get_logger().info(f"Arrived at waypoint {self.current_waypoint_index}. Hovering for 2 seconds.")
                self.state = "HOVERING_AT_WAYPOINT"
                self.hover_start_time = self.get_clock().now()

        elif self.state == "HOVERING_AT_WAYPOINT":
            if self.hover_start_time is None:
                self.state = "MOVING_TO_WAYPOINT"
                return

            target_wp = self.drone_waypoints[self.current_waypoint_index]
            target_stare_idx = self.stare_indices[self.current_waypoint_index]
            target_stare_pos = self.stare_targets[target_stare_idx]
            self.publish_position_setpoint(target_wp)
            self.point_gimbal_at_target(target_stare_pos)
            
            if self.get_clock().now() - self.hover_start_time > rclpy.duration.Duration(seconds=2):
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.drone_waypoints):
                    self.get_logger().info("🏁 모든 웨이포인트 방문 완료. 현재 위치에서 호버링 시작.")
                    self.state = "MISSION_COMPLETE_HOVER"
                    # 미션 컨트롤에 랑데뷰 지점 도착 및 호버링 완료 신호 전송
                    self.send_mission_complete(4)  # DRONE_APPROACH_COMPLETE
                    self.send_mission_complete(5)  # DRONE_HOVER_COMPLETE
                else:
                    self.get_logger().info(f"Hover complete. Moving to next waypoint: {self.current_waypoint_index}")
                    self.state = "MOVING_TO_WAYPOINT"
                self.hover_start_time = None
        
        elif self.state == "MISSION_COMPLETE_HOVER":
            # 마지막 웨이포인트에서 계속 호버링 (무한 호버링)
            final_wp = self.drone_waypoints[-1]
            final_stare_idx = self.stare_indices[-1]
            final_stare_pos = self.stare_targets[final_stare_idx]
            self.publish_position_setpoint(final_wp)
            self.point_gimbal_at_target(final_stare_pos)
            self.get_logger().info("✈️ 미션 완료 - 마지막 웨이포인트에서 호버링 중...", throttle_duration_sec=10.0)
        
        elif self.state == "LANDING":
            self.get_logger().info("Landing command issued.", throttle_duration_sec=5)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = "LANDED"

        elif self.state == "LANDED":
            self.get_logger().info("Drone has landed. Shutting down state machine.", once=True)
            self.state_machine_timer.cancel()
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = WaypointMissionNode()
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

