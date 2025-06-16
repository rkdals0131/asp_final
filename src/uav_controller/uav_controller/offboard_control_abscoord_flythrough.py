#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy

import numpy as np
from scipy.interpolate import splprep, splev
import math
import threading
import sys
import copy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# --- TF2 관련 모듈 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TrajectoryGenerator:
    """
    웨이포인트로부터 3D 스플라인 궤적(위치, 속도, 가속도)을 생성하고,
    RViz 시각화를 위한 마커를 제공하는 헬퍼 클래스.
    """
    def __init__(self, waypoints, logger):
        self.logger = logger
        self.waypoints = np.array(waypoints)
        self.tck = None
        self.total_path_length = 0
        self.trajectory = None  # Stores time, pos, vel, acc

        if len(self.waypoints) < 2:
            self.logger.error("At least 2 waypoints are required.")
            return

        self._generate_spline()
        self._calculate_path_length()

    def _generate_spline(self):
        """scipy를 사용하여 웨이포인트를 통과하는 3차 스플라인을 생성합니다."""
        try:
            self.tck, _ = splprep([self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2]], s=0, k=3)
            self.logger.info(f"Successfully generated a 3D spline through {len(self.waypoints)} waypoints.")
        except Exception as e:
            self.logger.error(f"Failed to generate spline: {e}")

    def _calculate_path_length(self):
        """생성된 스플라인의 총 길이를 수치적으로 계산합니다."""
        if self.tck is None: return
        u = np.linspace(0, 1, 1000)
        points = np.array(splev(u, self.tck)).T
        self.total_path_length = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        self.logger.info(f"Total path length: {self.total_path_length:.2f} meters.")

    def generate_trajectory(self, max_speed, max_acceleration):
        """
        사다리꼴 속도 프로파일을 적용하여 시간에 따른 궤적(Trajectory)을 생성합니다.
        """
        if self.total_path_length <= 1e-6:
            self.logger.warn("Total path length is near zero. No trajectory generated.")
            self.trajectory = []
            return self.trajectory, 0.0

        t_accel = max_speed / max_acceleration
        d_accel = 0.5 * max_acceleration * t_accel**2

        if 2 * d_accel >= self.total_path_length:
            t_accel = math.sqrt(self.total_path_length / max_acceleration)
            max_speed_actual = max_acceleration * t_accel
            t_cruise = 0.0
            d_cruise = 0.0
        else:
            d_cruise = self.total_path_length - 2 * d_accel
            t_cruise = d_cruise / max_speed
            max_speed_actual = max_speed

        t_total = 2 * t_accel + t_cruise
        self.logger.info(f"Estimated mission duration: {t_total:.2f} seconds.")

        num_samples = int(t_total * 50) if t_total > 0 else 0
        if num_samples == 0:
            self.logger.warn("Trajectory duration is zero, no points generated.")
            self.trajectory = []
            return self.trajectory, t_total
            
        time_samples = np.linspace(0, t_total, num_samples)
        self.trajectory = []

        distance_so_far = 0.0
        u = 0.0

        for t in time_samples:
            if t < t_accel:
                speed = max_acceleration * t
                distance = 0.5 * max_acceleration * t**2
            elif t < t_accel + t_cruise:
                speed = max_speed_actual
                distance = d_accel + max_speed_actual * (t - t_accel)
            else:
                time_in_decel = t - (t_accel + t_cruise)
                speed = max_speed_actual - max_acceleration * time_in_decel
                distance = d_accel + d_cruise + (max_speed_actual * time_in_decel - 0.5 * max_acceleration * time_in_decel**2)
            
            while distance_so_far < distance and u < 1.0:
                 u_next = u + 0.0001
                 if u_next > 1.0: u_next = 1.0
                 p1 = splev(u, self.tck)
                 p2 = splev(u_next, self.tck)
                 distance_so_far += np.linalg.norm(np.array(p2) - np.array(p1))
                 u = u_next

            pos = np.array(splev(u, self.tck, der=0))
            path_tangent_vec = np.array(splev(u, self.tck, der=1))
            ds_du = np.linalg.norm(path_tangent_vec)
            path_tangent_dir = path_tangent_vec / ds_du if ds_du > 1e-6 else np.array([1.0, 0.0, 0.0])
            vel = path_tangent_dir * speed
            
            if len(self.trajectory) > 0:
                prev_vel = self.trajectory[-1]['vel']
                dt = t - self.trajectory[-1]['t']
                acc = (vel - prev_vel) / dt if dt > 1e-6 else self.trajectory[-1]['acc']
            else:
                acc = path_tangent_dir * max_acceleration

            self.trajectory.append({'t': t, 'pos': pos, 'vel': vel, 'acc': acc})

        return self.trajectory, t_total

    def get_trajectory_at(self, mission_time):
        """주어진 미션 시간에 해당하는 궤적 포인트를 반환합니다."""
        if not self.trajectory: return None
        if mission_time > self.trajectory[-1]['t']: return self.trajectory[-1]
        idx = min(range(len(self.trajectory)), key=lambda i: abs(self.trajectory[i]['t'] - mission_time))
        return self.trajectory[idx]

    def get_path_visualization_marker(self):
        """RViz 시각화를 위한 경로 마커(Line Strip)를 생성합니다."""
        if self.tck is None or not self.trajectory: return None
        
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        # 스탬프는 호출하는 쪽에서 설정
        path_marker.ns = "spline_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.3
        path_marker.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0)
        path_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

        for point in self.trajectory:
            p = Point()
            p.x, p.y, p.z = point['pos']
            path_marker.points.append(p)
        
        return path_marker

class SplineMissionNode(Node):
    """
    스플라인 궤적을 생성하고 추종하며, Stare 기능을 수행하는 메인 노드.
    """
    def __init__(self):
        super().__init__('spline_mission_node')
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
            depth=10 # 여러 마커를 한번에 보내기 위해 깊이 증가
        )

        # --- 퍼블리셔/서브스크라이버 설정 ---
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.state_publisher = self.create_publisher(String, "/drone/state", 10)
        self.visual_marker_publisher = self.create_publisher(MarkerArray, "/mission_visuals", visual_qos_profile)

        self.local_position_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile)
        self.attitude_subscriber = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile)

        # --- TF 설정 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 미션 정의 ---
        mission_definition = [
            (-100, 80, 20, 0), (-80, 80, 30, 1), (-65, 83, 25, 2),
            (-80, 100, 20, 3), (-90, 103, 25, 4), (-105, 95, 30, 5),
            (-63, 100, 10, 6)
        ]
        self.drone_waypoints = np.array([p[0:3] for p in mission_definition])
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
        self.drone_frame_id = "x500_gimbal_0" # TF 조회를 위한 프레임 ID
        self.gimbal_camera_frame_id = "x500_gimbal_0/camera_link" # 짐벌 화살표 마커를 위한 프레임 ID
        self.handshake_counter = 0
        self.handshake_duration = 15

        # --- 궤적 생성 ---
        self.trajectory_generator = TrajectoryGenerator(self.drone_waypoints, self.get_logger())
        self.trajectory, self.mission_duration = self.trajectory_generator.generate_trajectory(max_speed=15.0, max_acceleration=4.0)
        self.mission_start_time = None
        self.mission_time = 0.0
        self.map_frame_offset = None
        
        # --- 메인 루프 및 커맨드 입력 ---
        self.state_machine_timer = self.create_timer(0.02, self.run_state_machine) # 50Hz
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info("Spline Mission Controller is initialized and ready.")

    def command_input_loop(self):
        print("\n--- Spline Mission Command ---")
        print("  start   - Arm and start the mission")
        print("  land    - Force landing")
        print("------------------------------")
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == "start":
                if self.state == "ARMED_IDLE":
                    self.get_logger().info("User command: START. Taking off and preparing for mission.")
                    self.state = "TAKING_OFF"
                else:
                    self.get_logger().warn(f"Cannot start mission from state: {self.state}")
            elif cmd == "land":
                 if self.state not in ["LANDING", "LANDED"]:
                     self.get_logger().warn("User command: LAND. Forcing landing.")
                     self.state = "LANDING"

    def local_position_callback(self, msg: VehicleLocalPosition): self.current_local_pos = msg
    def attitude_callback(self, msg: VehicleAttitude): self.current_attitude = msg

    def update_current_map_pose(self):
        try:
            # 베이스 링크의 위치를 조회
            trans = self.tf_buffer.lookup_transform('map', self.drone_frame_id, rclpy.time.Time())
            if self.current_map_pose is None: self.current_map_pose = PoseStamped()
            self.current_map_pose.pose.position.x = trans.transform.translation.x
            self.current_map_pose.pose.position.y = trans.transform.translation.y
            self.current_map_pose.pose.position.z = trans.transform.translation.z
            return True
        except TransformException as e:
            if self.state != "INIT": self.get_logger().warn(f"TF lookup failed for '{self.drone_frame_id}': {e}", throttle_duration_sec=1.0)
            return False

    def publish_offboard_control_mode(self, position=True, velocity=False, acceleration=False, attitude=False):
        msg = OffboardControlMode(
            position=position, velocity=velocity, acceleration=acceleration, attitude=attitude,
            timestamp=int(self.get_clock().now().nanoseconds / 1000)
        )
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand(command=command, timestamp=int(self.get_clock().now().nanoseconds / 1000), from_external=True, target_system=1, target_component=1)
        for i in range(1, 8): msg.__setattr__(f'param{i}', float(kwargs.get(f"param{i}", 0.0)))
        self.vehicle_command_publisher.publish(msg)
    
    def point_gimbal_at_target(self, target_enu_pos):
        """드론의 현재 ENU 위치와 목표 ENU 위치를 기반으로 짐벌을 제어합니다."""
        if self.current_map_pose is None or self.current_attitude is None: return

        drone_pos = self.current_map_pose.pose.position
        delta_x = target_enu_pos[0] - drone_pos.x
        delta_y = target_enu_pos[1] - drone_pos.y
        delta_z = target_enu_pos[2] - drone_pos.z
        
        # Pitch 계산
        distance_2d = math.sqrt(delta_x**2 + delta_y**2)
        pitch_rad = math.atan2(delta_z, distance_2d)
        
        # 지도 기준 Yaw 계산 (E-N-U frame: East=0, North=90)
        map_yaw_rad = math.atan2(delta_y, delta_x)
        
        # 드론의 현재 Yaw 계산 (E-N-U frame)
        q = self.current_attitude.q
        # Roll, Pitch, Yaw 순서의 ZYX 오일러 각
        drone_yaw_rad = math.atan2(2.0 * (q[3] * q[0] + q[1] * q[2]), 1.0 - 2.0 * (q[0]**2 + q[1]**2))

        # 짐벌에 필요한 상대 Yaw 계산 (타겟의 절대 Yaw - 드론의 절대 Yaw)
        relative_yaw_rad = map_yaw_rad - drone_yaw_rad
        # -PI ~ PI 범위로 정규화
        while relative_yaw_rad > math.pi: relative_yaw_rad -= 2.0 * math.pi
        while relative_yaw_rad < -math.pi: relative_yaw_rad += 2.0 * math.pi

        # VEHICLE_CMD_DO_MOUNT_CONTROL 명령 전송
        # param7=1.0: MAV_MOUNT_MODE_YAW_BODY (yaw가 기체 기준 상대각)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, 
            param1=math.degrees(pitch_rad),       # Pitch
            param3=math.degrees(relative_yaw_rad),# Yaw
            param7=1.0                            # Mount mode
        )

    def publish_mission_visuals(self):
        """RViz 시각화를 위한 모든 마커(경로, 타겟, 짐벌 방향)를 생성하고 게시합니다."""
        master_marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. 스플라인 경로 마커
        path_marker = self.trajectory_generator.get_path_visualization_marker()
        if path_marker:
            path_marker.header.stamp = now
            master_marker_array.markers.append(path_marker)

        # 2. Stare 타겟 마커 (구, 실린더, 텍스트)
        for i, target in enumerate(self.stare_targets):
            # 구 마커
            sphere = Marker()
            sphere.header.frame_id = "map"; sphere.header.stamp = now
            sphere.ns = "stare_targets_spheres"; sphere.id = i
            sphere.type = Marker.SPHERE; sphere.action = Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = target
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0
            sphere.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.8)
            sphere.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            master_marker_array.markers.append(sphere)

            # 텍스트 마커
            text = Marker()
            text.header.frame_id = "map"; text.header.stamp = now
            text.ns = "stare_targets_labels"; text.id = i
            text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = target[0], target[1], target[2] + 2.0
            text.pose.orientation.w = 1.0
            text.scale.z = 1.5
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"Target {i}"
            text.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            master_marker_array.markers.append(text)

        # 3. 짐벌 방향 화살표 마커
        gimbal_arrow = Marker()
        # camera_link 프레임은 카메라 렌즈를 기준으로 하므로, 화살표는 이 프레임에서 바로 그립니다.
        gimbal_arrow.header.frame_id = self.gimbal_camera_frame_id
        gimbal_arrow.header.stamp = now
        gimbal_arrow.ns = "gimbal_direction_arrow"
        gimbal_arrow.id = 0
        gimbal_arrow.type = Marker.ARROW
        gimbal_arrow.action = Marker.ADD
        
        # 화살표는 camera_link의 원점(0,0,0)에서 시작하여 -X축 방향으로 뻗어나갑니다.
        start_point = Point(x=0.0, y=0.0, z=0.0)
        end_point = Point(x=-2.5, y=0.0, z=0.0) # -X 방향으로 2.5m 길이
        gimbal_arrow.points.append(start_point)
        gimbal_arrow.points.append(end_point)
        
        gimbal_arrow.scale.x = 0.2  # 화살촉 몸통 직경
        gimbal_arrow.scale.y = 0.4  # 화살촉 머리 직경
        gimbal_arrow.color = ColorRGBA(r=0.9, g=0.1, b=0.9, a=1.0) # 마젠타색
        gimbal_arrow.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
        master_marker_array.markers.append(gimbal_arrow)

        # 최종 마커 배열 퍼블리시
        self.visual_marker_publisher.publish(master_marker_array)

    def check_arrival(self, target_pos, tolerance=1.5):
        if self.current_map_pose is None: return False
        pos = self.current_map_pose.pose.position
        return math.sqrt((pos.x - target_pos[0])**2 + (pos.y - target_pos[1])**2 + (pos.z - target_pos[2])**2) < tolerance

    def run_state_machine(self):
        # 1. 드론의 현재 위치(map frame) 업데이트
        if not self.update_current_map_pose() or self.current_local_pos is None:
            return

        # 2. RViz 시각화 업데이트
        self.publish_mission_visuals()

        # 3. 상태 메시지 퍼블리시
        state_msg = String(data=self.state)
        self.state_publisher.publish(state_msg)

        # 4. 오프보드 모드 활성화 (필요시)
        if self.state not in ["LANDING", "LANDED", "INIT"]:
             self.publish_offboard_control_mode(True, True, True)

        # 5. 상태별 로직 수행
        if self.state == "INIT":
            self.get_logger().info("System ready, starting handshake.", once=True)
            self.state = "HANDSHAKE"

        elif self.state == "HANDSHAKE":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # Offboard mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # Arm
            self.handshake_counter += 1
            if self.handshake_counter > self.handshake_duration:
                self.get_logger().info("Arm command sent. Ready for 'start' command.")
                self.state = "ARMED_IDLE"
        
        elif self.state == "ARMED_IDLE":
            # 현재 위치를 유지하도록 setpoint 전송
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, self.current_local_pos.z], timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)

        elif self.state == "TAKING_OFF":
            # 첫 웨이포인트의 고도로 이륙
            takeoff_altitude = self.drone_waypoints[0][2]
            # map frame의 목표 고도를 local NED 프레임으로 변환하여 전송
            delta_z_map = takeoff_altitude - self.current_map_pose.pose.position.z
            target_z_ned = self.current_local_pos.z - delta_z_map
            
            sp_msg = TrajectorySetpoint(position=[self.current_local_pos.x, self.current_local_pos.y, target_z_ned], yaw=math.nan, timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)

            if abs(self.current_map_pose.pose.position.z - takeoff_altitude) < 1.0:
                self.get_logger().info("Takeoff complete. Moving to mission start point.")
                self.state = "PREPARE_MISSION"

        elif self.state == "PREPARE_MISSION":
            # 미션 시작점(첫 웨이포인트)으로 이동
            start_point = self.drone_waypoints[0]
            # map frame의 목표 위치를 local NED 프레임으로 변환
            delta_map_x = start_point[0] - self.current_map_pose.pose.position.x
            delta_map_y = start_point[1] - self.current_map_pose.pose.position.y
            delta_map_z = start_point[2] - self.current_map_pose.pose.position.z
            
            target_ned_x = self.current_local_pos.x + delta_map_y # map.y -> ned.x
            target_ned_y = self.current_local_pos.y + delta_map_x # map.x -> ned.y
            target_ned_z = self.current_local_pos.z - delta_map_z # map.z -> -ned.z
            
            sp_msg = TrajectorySetpoint(position=[target_ned_x, target_ned_y, target_ned_z], yaw=math.nan, timestamp=int(self.get_clock().now().nanoseconds / 1000))
            self.trajectory_setpoint_publisher.publish(sp_msg)

            if self.check_arrival(start_point):
                self.get_logger().info("Reached mission start point. Calculating frame offset...")
                # map frame과 local frame 간의 오프셋 계산 (미션 실행 중 좌표 변환에 사용)
                offset_x = self.current_map_pose.pose.position.x - self.current_local_pos.y
                offset_y = self.current_map_pose.pose.position.y - self.current_local_pos.x
                offset_z = self.current_map_pose.pose.position.z - (-self.current_local_pos.z)
                self.map_frame_offset = (offset_x, offset_y, offset_z)
                self.get_logger().info(f"Frame offset calculated: {self.map_frame_offset}")
                self.get_logger().info("Executing spline trajectory...")
                self.mission_start_time = self.get_clock().now()
                self.state = "EXECUTING_MISSION"

        elif self.state == "EXECUTING_MISSION":
            if self.mission_start_time is None or self.map_frame_offset is None: return

            self.mission_time = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
            if self.mission_time > self.mission_duration:
                self.get_logger().info("Spline trajectory complete.")
                self.state = "LANDING"
                return

            # 궤적에서 현재 시간의 setpoint 가져오기
            setpoint = self.trajectory_generator.get_trajectory_at(self.mission_time)
            if setpoint is None: return

            # ENU 좌표계의 궤적을 NED 좌표계로 변환하여 PX4로 전송
            pos_enu, vel_enu, acc_enu = setpoint['pos'], setpoint['vel'], setpoint['acc']
            offset_x, offset_y, offset_z = self.map_frame_offset
            pos_ned = [pos_enu[1] - offset_y, pos_enu[0] - offset_x, -(pos_enu[2] - offset_z)]
            vel_ned = [vel_enu[1], vel_enu[0], -vel_enu[2]]
            acc_ned = [acc_enu[1], acc_enu[0], -acc_enu[2]]

            msg = TrajectorySetpoint()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.position = [float(p) for p in pos_ned]
            msg.velocity = [float(v) for v in vel_ned]
            msg.acceleration = [float(a) for a in acc_ned]
            msg.yaw = math.atan2(vel_enu[1], vel_enu[0])
            self.trajectory_setpoint_publisher.publish(msg)

            # --- 짐벌 제어 (Stare) 로직 ---
            # 현재 궤적의 진행률에 따라 stare할 타겟 결정
            progress = self.mission_time / self.mission_duration if self.mission_duration > 0 else 0
            waypoint_segment = int(progress * (len(self.drone_waypoints) - 1))
            stare_target_idx = self.stare_indices[waypoint_segment]
            self.point_gimbal_at_target(self.stare_targets[stare_target_idx])
            
            if waypoint_segment != getattr(self, 'last_segment', -1):
                self.get_logger().info(f"Entered segment {waypoint_segment}, now staring at target #{stare_target_idx}")
                self.last_segment = waypoint_segment

        elif self.state == "LANDING":
            self.get_logger().info("Mission complete. Landing at final destination.", throttle_duration_sec=5)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = "LANDED"

        elif self.state == "LANDED":
            self.get_logger().info("Drone has landed. Shutting down state machine.", once=True)
            self.state_machine_timer.cancel()
            pass

def main(args=None):
    rclpy.init(args=args)
    mission_node = SplineMissionNode()
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
