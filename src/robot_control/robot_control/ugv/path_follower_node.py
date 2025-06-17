#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from mission_admin_interfaces.srv import MissionComplete

import tf2_ros
from tf2_ros import TransformException

import math
import numpy as np
from scipy.interpolate import splprep, splev
import threading
import sys
import os
import yaml

class PathFollowerNode(Node):
    """
    S-Curve 속도 프로파일 기반 경로 추종 노드
    - 드론 탑재 여부에 따라 저크(Jerk)와 가속도를 동적으로 제어하여 최적화된 주행 구현
    - 경로 완료 시 미션 컨트롤에 신호 전송 (경로의 의미는 해석하지 않음)
    """
    def __init__(self):
        super().__init__('path_follower_advanced')

        # === 핵심 파라미터 ===
        self.declare_parameter('max_jerk_with_drone', 1.0,
            ParameterDescriptor(description="드론 탑재 시 최대 저크 (m/s³)"))
        self.declare_parameter('max_jerk_default', 3.0,
            ParameterDescriptor(description="기본(드론 미탑재) 최대 저크 (m/s³)"))
        self.declare_parameter('max_accel_with_drone', 0.5,
            ParameterDescriptor(description="드론 탑재 시 최대 종방향 가속도 (m/s²)"))
        self.declare_parameter('max_accel_default', 2.0,
            ParameterDescriptor(description="기본(드론 미탑재) 최대 종방향 가속도 (m/s²)"))

        self.declare_parameter('lookahead_k', 2.0, ParameterDescriptor(description="전방주시거리 비례상수"))
        self.declare_parameter('lookahead_min', 0.5, ParameterDescriptor(description="최소 전방주시거리 (m)"))
        self.declare_parameter('lookahead_max', 3.0, ParameterDescriptor(description="최대 전방주시거리 (m)"))
        self.declare_parameter('max_speed', 6.0, ParameterDescriptor(description="최대 주행 속도 (m/s)"))
        self.declare_parameter('min_speed', 0.5, ParameterDescriptor(description="최소 주행 속도 (m/s)"))
        self.declare_parameter('max_decel', 1.0, ParameterDescriptor(description="최대 종방향 감속도 (m/s²)"))
        self.declare_parameter('max_lateral_accel', 2.0, ParameterDescriptor(description="최대 횡방향 가속도 (m/s²)"))
        self.declare_parameter('waypoint_reach_threshold', 1.5, ParameterDescriptor(description="웨이포인트 도착 판단 반경 (m)"))
        self.declare_parameter('path_density', 0.1, ParameterDescriptor(description="경로점 생성 간격 (m)"))
        self.declare_parameter('map_frame', 'map', ParameterDescriptor(description="맵 TF 프레임"))
        self.declare_parameter('vehicle_base_frame', 'X1_asp/base_link', ParameterDescriptor(description="차량 기준 TF 프레임"))
        
        # === 경로 로딩 파라미터 ===
        self.declare_parameter('waypoint_file', '', 
            ParameterDescriptor(description="웨이포인트 파일 경로 (YAML/CSV). 비어있으면 기본 경로 사용"))
        self.declare_parameter('use_mission_ids', True, 
            ParameterDescriptor(description="미션 ID 기반 완료 신호 전송 여부"))

        # 파라미터 로딩
        self.MAX_JERK_WITH_DRONE = self.get_parameter('max_jerk_with_drone').value
        self.MAX_JERK_DEFAULT = self.get_parameter('max_jerk_default').value
        self.MAX_ACCEL_WITH_DRONE = self.get_parameter('max_accel_with_drone').value
        self.MAX_ACCEL_DEFAULT = self.get_parameter('max_accel_default').value
        
        self.LOOKAHEAD_K = self.get_parameter('lookahead_k').value
        self.LOOKAHEAD_MIN = self.get_parameter('lookahead_min').value
        self.LOOKAHEAD_MAX = self.get_parameter('lookahead_max').value
        self.MAX_SPEED = self.get_parameter('max_speed').value
        self.MIN_SPEED = self.get_parameter('min_speed').value
        self.MAX_DECEL = self.get_parameter('max_decel').value
        self.MAX_LATERAL_ACCEL = self.get_parameter('max_lateral_accel').value
        self.REACH_THRESHOLD = self.get_parameter('waypoint_reach_threshold').value
        self.PATH_DENSITY = self.get_parameter('path_density').value
        self.MAP_FRAME = self.get_parameter('map_frame').value
        self.VEHICLE_BASE_FRAME = self.get_parameter('vehicle_base_frame').value
        
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.use_mission_ids = self.get_parameter('use_mission_ids').value

        # 상태 변수
        self.vehicle_pose_map = None
        self.current_speed = 0.0
        self.current_max_accel = self.MAX_ACCEL_WITH_DRONE
        self.current_max_jerk = self.MAX_JERK_WITH_DRONE
        self.is_waiting_for_go = True
        self.is_mission_paused = False
        self.is_mission_complete = False
        self.current_waypoint_idx = 0
        self.last_closest_idx = 0
        self.is_orienting = False

        # 경로 데이터
        self.raw_waypoints = self._load_waypoints()
        if not self.raw_waypoints:
            self.get_logger().error("❌ 웨이포인트 로드 실패. 노드 종료.")
            return
            
        self.main_path_points = self._generate_path_points_from_list(self.raw_waypoints)
        self.full_path_points = []
        self.full_target_velocities = []

        # === 미션 완료 신호 매핑 ===
        # 미션의 구체적 의미를 모르고, 단순히 웨이포인트 인덱스별로 신호만 전송
        self.waypoint_mission_mapping = {
            2: 1,  # 3번째 웨이포인트 (인덱스 2) -> 미션 ID 1
            len(self.raw_waypoints) - 1: 3  # 마지막 웨이포인트 -> 미션 ID 3
        }

        # TF 및 통신
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers / Subscribers / Timers / Threads
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/X1_asp/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/generated_path', latched_qos)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', latched_qos)
        self.speed_marker_pub = self.create_publisher(MarkerArray, '/speed_markers', latched_qos)
        self.lookahead_marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)
        self.state_pub = self.create_publisher(String, '/vehicle/state', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/model/X1/odometry', self.odom_callback, 10)
        self.mission_command_sub = self.create_subscription(String, '/ugv/mission_command', self.mission_command_callback, 10)
        
        # 미션 컨트롤 서비스 클라이언트
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.path_update_timer = self.create_timer(0.2, self.update_full_path_and_velocity)
        self.state_timer = self.create_timer(0.1, self.publish_state)
        self.input_thread = threading.Thread(target=self._command_input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("🚗 S-Curve Path Follower v4.0 초기화 완료")
        self.get_logger().info(f"   - 웨이포인트 파일: {self.waypoint_file if self.waypoint_file else '기본 경로'}")
        self.get_logger().info(f"   - 로드된 웨이포인트: {len(self.raw_waypoints)}개")
        self.get_logger().info("   - 현재 모드: [With Drone]. 'go' 명령 대기 중")

    def odom_callback(self, msg: Odometry):
        self.current_speed = msg.twist.twist.linear.x

    def mission_command_callback(self, msg: String):
        """미션 컨트롤 대시보드로부터 명령 수신"""
        command = msg.data.lower()
        if command == 'go' and self.is_waiting_for_go:
            self.is_waiting_for_go = False
            self.path_update_timer.cancel()
            self.get_logger().info("🚀 미션 컨트롤로부터 GO 명령 수신. 미션 시작!")
        elif command == 'resume' and self.is_mission_paused:
            self._resume_mission()
        elif command == 'stop':
            self._stop_vehicle()
            self.is_mission_paused = True
            self.get_logger().info("⛔ 미션 컨트롤로부터 STOP 명령 수신")

    def publish_state(self):
        """현재 상태를 퍼블리시"""
        if self.is_waiting_for_go:
            state = "WAITING_FOR_GO"
        elif self.is_mission_paused:
            state = "PAUSED"
        elif self.is_mission_complete:
            state = "COMPLETE"
        elif self.is_orienting:
            state = "ORIENTING"
        else:
            state = "MOVING"
        
        self.state_pub.publish(String(data=state))

    def send_mission_complete(self, mission_id: int):
        """미션 완료 신호를 미션 컨트롤에 전송"""
        if not self.use_mission_ids:
            self.get_logger().info(f"📡 미션 완료 신호 전송 비활성화됨 (ID: {mission_id})")
            return
            
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

    def _update_vehicle_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.MAP_FRAME, self.VEHICLE_BASE_FRAME, rclpy.time.Time())
            pos = trans.transform.translation
            quat = trans.transform.rotation
            _, _, yaw = self._quat_to_euler(quat)
            self.vehicle_pose_map = (pos.x, pos.y, yaw)
            return True
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.VEHICLE_BASE_FRAME} to {self.MAP_FRAME}: {ex}', throttle_duration_sec=1.0)
            return False

    def control_loop(self):
        if not self._update_vehicle_pose():
            self._stop_vehicle()
            return

        if self.is_orienting:
            if self._orient_vehicle_for_next_segment():
                self.is_orienting = False
                self.get_logger().info("✅ Vehicle oriented. Continuing mission.")
            return

        if self.is_waiting_for_go or self.is_mission_complete or self.is_mission_paused:
            # 디버깅: 왜 정지했는지 로그
            if self.is_mission_paused:
                self.get_logger().info("🔧 Debug: Vehicle stopped due to mission paused", throttle_duration_sec=5.0)
            self._stop_vehicle()
            return
            
        self._check_waypoint_arrival()
        if self.is_mission_paused or self.is_mission_complete:
            if self.is_mission_paused:
                self.get_logger().info("🔧 Debug: Mission paused after waypoint check", throttle_duration_sec=2.0)
            return

        current_x, current_y, current_yaw = self.vehicle_pose_map
        goal_idx, lookahead_dist = self._find_goal_point(current_x, current_y)
        if goal_idx is None or goal_idx >= len(self.full_target_velocities):
            self._stop_vehicle()
            return

        goal_x, goal_y = self.full_path_points[goal_idx]
        target_speed = self.full_target_velocities[goal_idx]
        alpha = math.atan2(goal_y - current_y, goal_x - current_x) - current_yaw
        effective_speed = max(self.current_speed, 0.1)
        angular_z = (2.0 * effective_speed * math.sin(alpha)) / lookahead_dist
        
        twist_msg = Twist()
        twist_msg.linear.x = float(target_speed)
        twist_msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist_msg)
        self._publish_lookahead_marker((goal_x, goal_y))

    def _command_input_loop(self):
        for line in sys.stdin:
            if not rclpy.ok():
                break
            cmd = line.strip().lower()
            if self.is_waiting_for_go and cmd == 'go':
                self.is_waiting_for_go = False
                self.path_update_timer.cancel()
                self.get_logger().info("🚀 Mission started! Mode: [With Drone]. Final path profile locked.")
            elif self.is_mission_paused and cmd == 'resume':
                self._resume_mission()
            else:
                self.get_logger().warn(f"Invalid command: '{cmd}' in current state.")

    def update_full_path_and_velocity(self):
        if not self.is_waiting_for_go or not self._update_vehicle_pose():
            return
            
        start_x, start_y, _ = self.vehicle_pose_map
        if not self.main_path_points:
            self.get_logger().warn("Main path is empty, cannot generate full path.")
            return

        goal_x, goal_y = self.main_path_points[0]
        initial_path = self._generate_straight_path((start_x, start_y), (goal_x, goal_y))
        
        self.full_path_points = initial_path + self.main_path_points
        
        # 마지막 웨이포인트의 타입에 따라 종료 속도 결정
        final_waypoint = self.raw_waypoints[-1]
        end_vel = 0.0 if len(final_waypoint) > 2 and final_waypoint[2] == 4 else 0.0  # 안전을 위해 항상 0으로 종료
        
        self.full_target_velocities = self._generate_scurve_velocity_profile(
            self.full_path_points, start_vel=self.current_speed, end_vel=end_vel)
        self._publish_path_visualization()

    def _check_waypoint_arrival(self):
        """웨이포인트 도달 확인 - 역할 단순화"""
        if self.current_waypoint_idx >= len(self.raw_waypoints):
            return
            
        waypoint = self.raw_waypoints[self.current_waypoint_idx]
        wp_x, wp_y = waypoint[0], waypoint[1]
        
        if self.vehicle_pose_map is None:
            return
            
        dist = math.hypot(self.vehicle_pose_map[0] - wp_x, self.vehicle_pose_map[1] - wp_y)
        
        # 특정 웨이포인트에서만 디버깅 로그 출력
        if self.current_waypoint_idx == 2:
            self.get_logger().info(f"🔍 Debug: WP{self.current_waypoint_idx} 거리체크 - 현재위치:({self.vehicle_pose_map[0]:.2f}, {self.vehicle_pose_map[1]:.2f}), 목표:({wp_x:.2f}, {wp_y:.2f}), 거리:{dist:.2f}m, 임계값:{self.REACH_THRESHOLD}m", throttle_duration_sec=2.0)
        
        if dist < self.REACH_THRESHOLD:
            self.get_logger().info(f"✅ Waypoint {self.current_waypoint_idx} reached (distance: {dist:.2f}m).")
            
            # 특정 웨이포인트에서만 처리 (기존 로직 유지)
            if len(waypoint) > 2:  # 미션 타입 정보가 있는 경우
                mission_type = waypoint[2]
                if mission_type == 2:  # 드론 이륙 지점
                    self.is_mission_paused = True
                    self.get_logger().info("🚁 Drone takeoff point reached. Vehicle paused.")
                    self.get_logger().info(f"🔧 Debug: is_mission_paused set to {self.is_mission_paused}")
                elif mission_type == 4:  # 최종 목적지
                    self.is_mission_complete = True
                    self.get_logger().info("🏁 Mission completed!")
            
            # 미션 완료 신호 전송 (매핑 테이블 사용)
            if self.current_waypoint_idx in self.waypoint_mission_mapping:
                mission_id = self.waypoint_mission_mapping[self.current_waypoint_idx]
                self.send_mission_complete(mission_id)
            
            self.current_waypoint_idx += 1
            self.last_closest_idx = self._find_closest_point_idx(self.vehicle_pose_map[0], self.vehicle_pose_map[1])

    def _resume_mission(self):
        self.get_logger().info("🚀 Switching to high-performance mode (Drone departed).")
        self.current_max_accel = self.MAX_ACCEL_DEFAULT
        self.current_max_jerk = self.MAX_JERK_DEFAULT

        remaining_waypoints = self.raw_waypoints[self.current_waypoint_idx:]
        if not remaining_waypoints:
            self.is_mission_complete = True
            self.get_logger().info("No more waypoints. Mission complete.")
            return
            
        self.main_path_points = self._generate_path_points_from_list(remaining_waypoints)
        self.update_full_path_and_velocity()
        
        self.is_mission_paused = False
        self.is_orienting = True

    def _find_goal_point(self, x, y):
        closest_idx = self._find_closest_point_idx(x, y)
        self.last_closest_idx = closest_idx
        lookahead_dist = np.clip(self.LOOKAHEAD_K * self.current_speed + self.LOOKAHEAD_MIN, self.LOOKAHEAD_MIN, self.LOOKAHEAD_MAX)
        for i in range(closest_idx, len(self.full_path_points)):
            if math.hypot(x - self.full_path_points[i][0], y - self.full_path_points[i][1]) >= lookahead_dist:
                return i, lookahead_dist
        return len(self.full_path_points) - 1, lookahead_dist

    def _find_closest_point_idx(self, x, y):
        if not self.full_path_points: return 0
        search_end = min(self.last_closest_idx + 200, len(self.full_path_points))
        path_segment = np.array(self.full_path_points[self.last_closest_idx:search_end])
        if len(path_segment) == 0: return self.last_closest_idx
        dists = np.linalg.norm(path_segment - np.array([x, y]), axis=1)
        return self.last_closest_idx + np.argmin(dists)

    def _orient_vehicle_for_next_segment(self):
        closest_idx = self._find_closest_point_idx(self.vehicle_pose_map[0], self.vehicle_pose_map[1])
        if closest_idx >= len(self.full_path_points) - 1: return True
        target_idx = min(closest_idx + int(1.0 / self.PATH_DENSITY), len(self.full_path_points) - 1)
        _, _, current_yaw = self.vehicle_pose_map
        target_x, target_y = self.full_path_points[target_idx]
        target_yaw = math.atan2(target_y - self.vehicle_pose_map[1], target_x - self.vehicle_pose_map[0])
        angle_diff = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_diff) < math.radians(5.0):
            self._stop_vehicle()
            return True
        twist_msg = Twist()
        twist_msg.angular.z = float(np.clip(angle_diff * 1.5, -1.0, 1.0))
        self.cmd_vel_pub.publish(twist_msg)
        return False

    def _generate_path_points_from_list(self, waypoints):
        if len(waypoints) < 2: return []
        wx = [p[0] for p in waypoints]
        wy = [p[1] for p in waypoints]
        tck, _ = splprep([wx, wy], s=0, per=False)
        path_len = np.sum(np.sqrt(np.diff(wx)**2 + np.diff(wy)**2))
        num_points = max(2, int(path_len / self.PATH_DENSITY))
        u_fine = np.linspace(0, 1, num_points)
        x_fine, y_fine = splev(u_fine, tck)
        return list(zip(x_fine, y_fine))

    def _generate_straight_path(self, start_pos, end_pos):
        dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
        if dist < self.PATH_DENSITY: return []
        num_points = max(2, int(dist / self.PATH_DENSITY))
        return list(zip(np.linspace(start_pos[0], end_pos[0], num_points), np.linspace(start_pos[1], end_pos[1], num_points)))

    def _generate_scurve_velocity_profile(self, path_points, start_vel=0.0, end_vel=0.0):
        if not path_points or len(path_points) < 2: return []
        num_points = len(path_points)
        distances = self._calculate_path_distances(path_points)
        velocity_limits = self._calculate_curvature_limited_velocities([p[0] for p in path_points], [p[1] for p in path_points])
        
        # 웨이포인트 기반 속도 제한 적용
        for i, waypoint in enumerate(self.raw_waypoints):
            wp_x, wp_y = waypoint[0], waypoint[1]
            closest_idx = min(range(num_points), key=lambda i: math.hypot(path_points[i][0] - wp_x, path_points[i][1] - wp_y))
            
            # 웨이포인트 타입별 속도 설정
            if len(waypoint) > 3:
                target_speed = waypoint[3]
                if len(waypoint) > 2 and waypoint[2] in [2, 4]:  # 정지 지점
                    wp_vel = 0.0
                elif target_speed < 0:  # 최대 속도
                    wp_vel = self.MAX_SPEED
                else:
                    wp_vel = np.clip(target_speed, self.MIN_SPEED, self.MAX_SPEED)
            else:
                wp_vel = self.MAX_SPEED
            
            velocity_limits[closest_idx] = min(velocity_limits[closest_idx], wp_vel)
        
        trapezoidal_profile = list(velocity_limits)
        trapezoidal_profile[0] = float(start_vel)
        for i in range(1, num_points):
            ds = distances[i] - distances[i-1]
            if ds < 1e-6:
                trapezoidal_profile[i] = trapezoidal_profile[i-1]
                continue
            max_reachable_vel = math.sqrt(trapezoidal_profile[i-1]**2 + 2 * self.current_max_accel * ds)
            trapezoidal_profile[i] = min(trapezoidal_profile[i], max_reachable_vel)

        trapezoidal_profile[-1] = float(end_vel)
        for i in range(num_points - 2, -1, -1):
            ds = distances[i+1] - distances[i]
            if ds < 1e-6:
                trapezoidal_profile[i] = trapezoidal_profile[i+1]
                continue
            max_braking_vel = math.sqrt(trapezoidal_profile[i+1]**2 + 2 * self.MAX_DECEL * ds)
            trapezoidal_profile[i] = min(trapezoidal_profile[i], max_braking_vel)
        
        final_profile = list(trapezoidal_profile)
        current_accel = 0.0
        final_profile[0] = float(start_vel)
        for i in range(1, num_points):
            ds = distances[i] - distances[i-1]
            if ds < 1e-6:
                final_profile[i] = final_profile[i-1]
                continue
            v_prev, v_guide = final_profile[i-1], trapezoidal_profile[i]
            
            req_accel = (v_guide**2 - v_prev**2) / (2 * ds) if ds > 1e-6 else 0.0
            lim_accel = np.clip(req_accel, -self.MAX_DECEL, self.current_max_accel)
            
            dt = ds / max(v_prev, 0.1)
            max_accel_change = self.current_max_jerk * dt
            final_accel = np.clip(lim_accel, current_accel - max_accel_change, current_accel + max_accel_change)
            
            v_from_jerk = math.sqrt(max(0, v_prev**2 + 2 * final_accel * ds))
            final_profile[i] = min(v_from_jerk, trapezoidal_profile[i])
            current_accel = (final_profile[i]**2 - v_prev**2) / (2 * ds) if ds > 1e-6 else 0.0
        final_profile[-1] = float(end_vel)
        return final_profile
    
    def _calculate_curvature_limited_velocities(self, x, y):
        curvatures = self._calculate_curvature(x, y)
        velocities = []
        for k in curvatures:
            abs_k = abs(k)
            if abs_k < 1e-6:
                max_v = self.MAX_SPEED
            else:
                max_v = math.sqrt(self.MAX_LATERAL_ACCEL / abs_k)
            velocities.append(np.clip(max_v, 0.0, self.MAX_SPEED))
        return velocities
    
    def _calculate_path_distances(self, path_points):
        distances = [0.0]
        for i in range(1, len(path_points)):
            dist = math.hypot(path_points[i][0] - path_points[i-1][0], path_points[i][1] - path_points[i-1][1])
            distances.append(distances[-1] + dist)
        return distances

    def _calculate_curvature(self, x, y):
        dx, dy = np.gradient(x), np.gradient(y)
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        return (dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)

    # === [REFACTORED] Visualization Functions ===
    def _publish_path_visualization(self):
        if not self.full_path_points:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.MAP_FRAME
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.full_path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.orientation.w = 1.0
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self._publish_waypoint_markers()
        self._publish_speed_markers()

    def _publish_waypoint_markers(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.raw_waypoints):
            x, y = waypoint[0], waypoint[1]
            marker = Marker()
            marker.header.frame_id = self.MAP_FRAME
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.5
            
            # 웨이포인트 타입별 색상 (타입 정보가 있는 경우)
            if len(waypoint) > 2:
                mission_type = waypoint[2]
                if mission_type == 2:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # Yellow
                elif mission_type == 4:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # Red
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0  # Blue
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0  # Blue
                
            # 속도 정보에 따른 색상 조정
            if len(waypoint) > 3:
                target_speed = waypoint[3]
                if 0 < target_speed <= 1.0:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0  # Orange
                    
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        self.waypoint_marker_pub.publish(marker_array)

    def _publish_speed_markers(self):
        marker_array = MarkerArray()
        step = max(1, int(0.5 / self.PATH_DENSITY))

        # Clear previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.speed_marker_pub.publish(marker_array)
        marker_array.markers.clear()

        for i in range(0, len(self.full_path_points), step):
            if i >= len(self.full_target_velocities):
                break
            
            point = self.full_path_points[i]
            vel = self.full_target_velocities[i]
            
            # Cylinder marker for speed magnitude
            cyl_height = max(0.2, float(vel * 0.5))
            cylinder = Marker()
            cylinder.header.frame_id = self.MAP_FRAME
            cylinder.ns = "speed_cylinders"
            cylinder.id = i
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position.x = point[0]
            cylinder.pose.position.y = point[1]
            cylinder.pose.position.z = cyl_height / 2.0
            cylinder.pose.orientation.w = 1.0
            cylinder.scale.x = cylinder.scale.y = 0.15
            cylinder.scale.z = cyl_height
            
            if vel <= 1.0:
                cylinder.color.r, cylinder.color.g, cylinder.color.b = 0.0, 0.5, 1.0 # Blue
            elif vel <= self.MAX_SPEED * 0.6:
                cylinder.color.r, cylinder.color.g, cylinder.color.b = 0.0, 1.0, 0.0 # Green
            else:
                cylinder.color.r, cylinder.color.g, cylinder.color.b = 1.0, 0.0, 0.0 # Red
            cylinder.color.a = 0.6
            marker_array.markers.append(cylinder)

            # Text marker for speed value
            text = Marker()
            text.header.frame_id = self.MAP_FRAME
            text.ns = "speed_text"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point[0]
            text.pose.position.y = point[1]
            text.pose.position.z = cyl_height + 0.3
            text.pose.orientation.w = 1.0
            text.text = f"{vel:.1f}"
            text.scale.z = 0.3
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            marker_array.markers.append(text)
            
        self.speed_marker_pub.publish(marker_array)

    def _publish_lookahead_marker(self, goal_point):
        marker = Marker()
        marker.header.frame_id = self.MAP_FRAME
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_point[0]
        marker.pose.position.y = goal_point[1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.8
        self.lookahead_marker_pub.publish(marker)

    def _load_waypoints(self):
        """웨이포인트 로드 - 파일 또는 기본값"""
        # 파일에서 로드 시도
        if self.waypoint_file:
            try:
                waypoints = self._load_waypoints_from_file(self.waypoint_file)
                if waypoints:
                    self.get_logger().info(f"✅ 웨이포인트 파일 로드 성공: {self.waypoint_file}")
                    return waypoints
                else:
                    self.get_logger().warn(f"⚠️ 웨이포인트 파일이 비어있음: {self.waypoint_file}")
            except Exception as e:
                self.get_logger().error(f"❌ 웨이포인트 파일 로드 실패: {e}")
                self.get_logger().info("기본 웨이포인트 사용")
        
        # 기본 웨이포인트 반환
        return [
            (-130.04, 51.88, 1, -1.0),
            (-132.71, 58.04, 1, -1.0),
            (-132.87, 64.00, 2, 0.0),  # 드론 이륙 지점
            (-129.23, 69.36, 3, -1.0), (-120.85, 73.20, 3, -1.0), (-117.45, 73.15, 3, -1.0),
            (-113.63, 72.64, 3, -1.0), (-104.97, 77.01, 3, -1.0), (-94.75, 84.41, 3, -1.0),
            (-91.71, 86.98, 3, -1.0), (-80.82, 97.95, 3, -1.0), (-76.74, 99.61, 3, 1.0),
            (-73.90, 98.63, 3, 1.0), (-72.13, 98.65, 3, 1.0), (-62.96, 99.09, 4, 0.0)  # 최종 목적지
        ]

    def _load_waypoints_from_file(self, file_path):
        """파일에서 웨이포인트 로드 (YAML 또는 CSV)"""
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {file_path}")
        
        _, ext = os.path.splitext(file_path.lower())
        
        if ext == '.yaml' or ext == '.yml':
            return self._load_waypoints_from_yaml(file_path)
        elif ext == '.csv':
            return self._load_waypoints_from_csv(file_path)
        else:
            raise ValueError(f"지원하지 않는 파일 형식: {ext}")

    def _load_waypoints_from_yaml(self, file_path):
        """YAML 파일에서 웨이포인트 로드"""
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        
        if 'waypoints' not in data:
            raise ValueError("YAML 파일에 'waypoints' 키가 없습니다")
        
        waypoints = []
        for wp in data['waypoints']:
            x = float(wp['x'])
            y = float(wp['y'])
            mission_type = wp.get('mission_type', 1)
            target_speed = wp.get('target_speed', -1.0)
            waypoints.append((x, y, mission_type, target_speed))
        
        return waypoints

    def _load_waypoints_from_csv(self, file_path):
        """CSV 파일에서 웨이포인트 로드"""
        import csv
        waypoints = []
        
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            # 헤더 건너뛰기 (첫 번째 행이 헤더인 경우)
            first_row = next(reader)
            if not first_row[0].replace('-', '').replace('.', '').isdigit():
                pass  # 헤더였음
            else:
                # 첫 번째 행이 데이터였음
                x, y = float(first_row[0]), float(first_row[1])
                mission_type = int(first_row[2]) if len(first_row) > 2 else 1
                target_speed = float(first_row[3]) if len(first_row) > 3 else -1.0
                waypoints.append((x, y, mission_type, target_speed))
            
            # 나머지 행 처리
            for row in reader:
                if len(row) < 2:
                    continue
                x, y = float(row[0]), float(row[1])
                mission_type = int(row[2]) if len(row) > 2 else 1
                target_speed = float(row[3]) if len(row) > 3 else -1.0
                waypoints.append((x, y, mission_type, target_speed))
        
        return waypoints

    def _quat_to_euler(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return 0, 0, math.atan2(t3, t4)

    def _stop_vehicle(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
