#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

import math
import numpy as np
from scipy.interpolate import splprep, splev
import threading
import sys

class PathFollowerNode(Node):
    """
    스플라인 기반 경로 추종 노드 - 부드러운 속도 프로파일링과 커브 인식 기능
    """
    def __init__(self):
        super().__init__('path_follower_advanced')

        # === 핵심 파라미터 ===
        self.declare_parameter('lookahead_k', 2.0)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_accel', 0.8,
            ParameterDescriptor(description="최대 종방향 가속도 (m/s²)"))
        self.declare_parameter('max_decel', 1.0,
            ParameterDescriptor(description="최대 종방향 감속도 (m/s²)"))
        self.declare_parameter('min_accel', 0.1,
            ParameterDescriptor(description="최소 종방향 가속도 (m/s²) - 정상상태오차 방지"))
        self.declare_parameter('max_lateral_accel', 2.0,
            ParameterDescriptor(description="최대 횡방향 가속도 (m/s²) - 커브 속도 제한"))
        self.declare_parameter('waypoint_reach_threshold', 1.5)
        self.declare_parameter('path_density', 0.2)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('vehicle_base_frame', 'X1_asp/base_link')

        # 파라미터 로딩
        self.LOOKAHEAD_K = self.get_parameter('lookahead_k').value
        self.LOOKAHEAD_MIN = self.get_parameter('lookahead_min').value
        self.LOOKAHEAD_MAX = self.get_parameter('lookahead_max').value
        self.MAX_SPEED = self.get_parameter('max_speed').value
        self.MIN_SPEED = self.get_parameter('min_speed').value
        self.MAX_ACCEL = self.get_parameter('max_accel').value
        self.MAX_DECEL = self.get_parameter('max_decel').value
        self.MIN_ACCEL = self.get_parameter('min_accel').value
        self.MAX_LATERAL_ACCEL = self.get_parameter('max_lateral_accel').value
        self.REACH_THRESHOLD = self.get_parameter('waypoint_reach_threshold').value
        self.PATH_DENSITY = self.get_parameter('path_density').value
        self.MAP_FRAME = self.get_parameter('map_frame').value
        self.VEHICLE_BASE_FRAME = self.get_parameter('vehicle_base_frame').value

        # 상태 변수
        self.vehicle_pose_map = None
        self.current_speed = 0.0
        self.is_waiting_for_go = True
        self.is_mission_paused = False
        self.is_mission_complete = False
        self.current_waypoint_idx = 0
        self.last_closest_idx = 0
        self.is_orienting = False

        # 경로 데이터
        self.raw_waypoints = self._load_waypoints()
        self.main_path_points, self.main_target_velocities = self._generate_main_path()
        self.full_path_points = []
        self.full_target_velocities = []

        # TF 및 통신
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/model/X1_asp/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/generated_path', latched_qos)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', latched_qos)
        self.speed_marker_pub = self.create_publisher(MarkerArray, '/speed_markers', latched_qos)
        self.lookahead_marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/model/X1/odometry', self.odom_callback, 10)
        
        # Timers
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.initial_path_timer = self.create_timer(0.2, self._update_initial_path)

        # 사용자 입력 스레드
        self.input_thread = threading.Thread(target=self._command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("🚗 Path follower initialized. Type 'go' to start mission.")

    def odom_callback(self, msg: Odometry):
        """오도메트리에서 현재 속도 추출"""
        self.current_speed = msg.twist.twist.linear.x

    def _update_vehicle_pose(self):
        """TF를 통해 차량 위치 업데이트"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.MAP_FRAME, self.VEHICLE_BASE_FRAME, rclpy.time.Time())
            pos = trans.transform.translation
            quat = trans.transform.rotation
            _, _, yaw = self._quat_to_euler(quat)
            self.vehicle_pose_map = (pos.x, pos.y, yaw)
            return True
        except TransformException:
            return False

    def control_loop(self):
        """메인 제어 루프"""
        if not self._update_vehicle_pose():
            self._stop_vehicle()
            return

        # 방향 전환 모드 (resume 후)
        if self.is_orienting:
            if self._orient_vehicle_for_next_segment():
                self.is_orienting = False
                self.get_logger().info("✅ Vehicle oriented. Continuing mission.")
            return

        # 대기/정지 상태 처리
        if self.is_waiting_for_go or self.is_mission_complete or self.is_mission_paused:
            self._stop_vehicle()
            return

        # 웨이포인트 도착 확인
        self._check_waypoint_arrival()
        if self.is_mission_paused or self.is_mission_complete:
            return

        # Pure Pursuit 제어
        current_x, current_y, current_yaw = self.vehicle_pose_map
        goal_idx, lookahead_dist = self._find_goal_point(current_x, current_y)
        
        if goal_idx is None:
            self._stop_vehicle()
            return

        # 목표점과 제어 명령 계산
        goal_x, goal_y = self.full_path_points[goal_idx]
        target_speed = self.full_target_velocities[goal_idx]
        
        alpha = math.atan2(goal_y - current_y, goal_x - current_x) - current_yaw
        angular_z = (2.0 * self.current_speed * math.sin(alpha)) / lookahead_dist
        
        # 제어 명령 발행
        twist_msg = Twist()
        twist_msg.linear.x = target_speed
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

        # 시각화
        self._publish_lookahead_marker((goal_x, goal_y))

    def _command_input_loop(self):
        """사용자 명령 입력 처리"""
        for line in sys.stdin:
            if not rclpy.ok():
                break
            cmd = line.strip().lower()

            if self.is_waiting_for_go and cmd == 'go':
                self.is_waiting_for_go = False
                self.initial_path_timer.cancel()
                self.get_logger().info("🚀 Mission started!")
            elif self.is_mission_paused and cmd == 'resume':
                self._resume_mission()
            else:
                self.get_logger().warn(f"Invalid command: '{cmd}'")

    def _update_initial_path(self):
        """초기 경로 생성 및 갱신"""
        if not self.is_waiting_for_go or not self._update_vehicle_pose():
            return
        
        start_x, start_y, start_yaw = self.vehicle_pose_map
        goal_x, goal_y = self.raw_waypoints[0][:2]
        
        # 직선 경로 생성
        initial_path = self._generate_straight_path((start_x, start_y), (goal_x, goal_y))
        if not initial_path:
            return
        
        # 부드러운 속도 프로파일 생성
        end_velocity = self.main_target_velocities[0] if self.main_target_velocities else 0.0
        initial_velocities = self._generate_smooth_velocity_profile(
            initial_path, start_vel=self.current_speed, end_vel=end_velocity
        )

        # 전체 경로 결합
        self.full_path_points = initial_path + self.main_path_points
        self.full_target_velocities = initial_velocities + self.main_target_velocities

        # 시각화 업데이트
        self._publish_path_visualization()

    def _check_waypoint_arrival(self):
        """웨이포인트 도착 확인"""
        if self.current_waypoint_idx >= len(self.raw_waypoints):
            return
        
        wp_x, wp_y, wp_mission, wp_speed = self.raw_waypoints[self.current_waypoint_idx]
        dist = math.hypot(self.vehicle_pose_map[0] - wp_x, self.vehicle_pose_map[1] - wp_y)
        
        # 미션 타입별 도착 임계값 설정
        arrival_threshold = self.REACH_THRESHOLD
        
        if dist < arrival_threshold:
            self.get_logger().info(f"✅ Waypoint {self.current_waypoint_idx} reached (Mission: {wp_mission}, Target Speed: {wp_speed}, Distance: {dist:.2f}m)")
            
            if wp_mission == 2:  # 드론 이륙 대기
                # 물리적 가속도 제약을 활용한 정확한 정지로 30cm 이내 도달 성공
                self.is_mission_paused = True
                self.get_logger().info("🚁 Drone takeoff waiting point reached within 30cm. Vehicle accurately stopped.")
                print("\n>>> Enter 'resume' to continue after drone takeoff")
            elif wp_mission == 4:  # 미션 완료
                self.is_mission_complete = True
                self.get_logger().info("🏁 Mission completed!")
                
            self.current_waypoint_idx += 1
            self.last_closest_idx = self._find_closest_point_idx(
                self.vehicle_pose_map[0], self.vehicle_pose_map[1])

    def _resume_mission(self):
        """미션 재개"""
        self.get_logger().info("▶️ Resuming mission...")
        self.is_mission_paused = False
        self.is_orienting = True

    def _find_goal_point(self, x, y):
        """Pure Pursuit 목표점 찾기"""
        closest_idx = self._find_closest_point_idx(x, y)
        self.last_closest_idx = closest_idx
        
        # 동적 전방주시거리 계산
        lookahead_dist = np.clip(
            self.LOOKAHEAD_K * self.current_speed + self.LOOKAHEAD_MIN,
            self.LOOKAHEAD_MIN, self.LOOKAHEAD_MAX
        )
        
        # 전방주시거리만큼 떨어진 점 찾기
        for i in range(closest_idx, len(self.full_path_points)):
            dist = math.hypot(x - self.full_path_points[i][0], y - self.full_path_points[i][1])
            if dist >= lookahead_dist:
                return i, lookahead_dist
                
        return len(self.full_path_points) - 1, lookahead_dist

    def _find_closest_point_idx(self, x, y):
        """가장 가까운 경로점 인덱스 찾기"""
        min_dist = float('inf')
        closest_idx = self.last_closest_idx
        
        # 효율적인 탐색을 위해 제한된 범위에서 검색
        search_end = min(self.last_closest_idx + 200, len(self.full_path_points))
        for i in range(self.last_closest_idx, search_end):
            dist = math.hypot(x - self.full_path_points[i][0], y - self.full_path_points[i][1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx

    def _orient_vehicle_for_next_segment(self):
        """다음 구간을 향해 차량 방향 조정"""
        closest_idx = self._find_closest_point_idx(self.vehicle_pose_map[0], self.vehicle_pose_map[1])
        
        if closest_idx >= len(self.full_path_points) - 1:
            return True

        # 목표 방향 계산
        target_idx = min(closest_idx + int(1.0 / self.PATH_DENSITY), len(self.full_path_points) - 1)
        current_x, current_y, current_yaw = self.vehicle_pose_map
        target_x, target_y = self.full_path_points[target_idx]

        target_yaw = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = target_yaw - current_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 방향 정렬 완료 확인 (5도 허용오차)
        if abs(angle_diff) < math.radians(5.0):
            self._stop_vehicle()
            return True

        # 회전 제어
        twist_msg = Twist()
        twist_msg.angular.z = np.clip(angle_diff * 1.2, -1.0, 1.0)
        self.cmd_vel_pub.publish(twist_msg)
        
        return False

    # === 경로 생성 및 속도 프로파일링 ===
    
    def _generate_main_path(self):
        """메인 경로 생성 - 스플라인 보간과 웨이포인트 목표 속도 반영"""
        if len(self.raw_waypoints) < 2:
            return [], []
        
        # 스플라인 보간
        wx = [p[0] for p in self.raw_waypoints]
        wy = [p[1] for p in self.raw_waypoints]
        tck, _ = splprep([wx, wy], s=0, per=False)
        
        # 고밀도 경로점 생성
        path_len = np.sum(np.sqrt(np.diff(wx)**2 + np.diff(wy)**2))
        num_points = int(path_len / self.PATH_DENSITY)
        u_fine = np.linspace(0, 1, num_points)
        x_fine, y_fine = splev(u_fine, tck)
        path_points = list(zip(x_fine, y_fine))
        
        # 사전 속도 플래닝: 웨이포인트 제약 + 곡률 기반 횡방향 가속도 제한
        velocities = self._generate_physics_based_velocity_profile(path_points, wx, wy, x_fine, y_fine)
        
        self.get_logger().info(f"Generated main path: {len(path_points)} points")
        return path_points, velocities

    def _generate_straight_path(self, start_pos, end_pos):
        """직선 경로 생성"""
        start_x, start_y = start_pos
        end_x, end_y = end_pos
        dist = math.hypot(end_x - start_x, end_y - start_y)
        
        if dist < 0.1:
            return []

        num_points = max(2, int(dist / self.PATH_DENSITY))
        x_fine = np.linspace(start_x, end_x, num_points)
        y_fine = np.linspace(start_y, end_y, num_points)
        return list(zip(x_fine, y_fine))

    def _generate_smooth_velocity_profile(self, path_points, start_vel=0.0, end_vel=0.0):
        """부드러운 속도 프로파일 생성 - 물리적 가속도 기반"""
        if not path_points or len(path_points) < 2:
            return [max(start_vel, self.MIN_SPEED)] * len(path_points)
        
        # 경로 거리 계산
        distances = self._calculate_path_distances(path_points)
        total_distance = distances[-1]
        
        if total_distance < 0.1:
            return [max(start_vel, self.MIN_SPEED)] * len(path_points)
        
        velocities = []
        
        for i, s in enumerate(distances):
            # 정규화된 거리 (0~1)
            s_norm = s / total_distance
            
            # 물리적 가속도 기반 속도 계산
            if end_vel > start_vel:  # 가속
                # v² = v₀² + 2as를 활용한 가속
                vel_squared = start_vel**2 + 2 * self.MAX_ACCEL * s
                vel = math.sqrt(max(0, vel_squared))
                
                # 목표 속도를 넘지 않도록 제한
                if vel > end_vel:
                    # 목표 속도 달성 후 일정 속도 유지
                    vel = end_vel
                    
            else:  # 감속 또는 일정 속도
                # 3차 다항식으로 부드러운 속도 변화
                smooth_component = start_vel + (end_vel - start_vel) * (3 * s_norm**2 - 2 * s_norm**3)
                vel = smooth_component
            
            # MIN_SPEED와 MAX_SPEED 범위 적용
            vel = np.clip(vel, self.MIN_SPEED, self.MAX_SPEED)
            velocities.append(vel)
        
        # 종점 속도 보정
        if velocities:
            velocities[-1] = max(end_vel, self.MIN_SPEED) if end_vel > 0 else end_vel
            
        return velocities

    def _generate_physics_based_velocity_profile(self, path_points, wx, wy, x_fine, y_fine):
        """통합 물리 법칙 기반 속도 플래닝 - 종방향/횡방향 가속도 + 웨이포인트 제약 통합 처리"""
        if not path_points:
            return []
        
        # 1단계: 기본 정보 계산
        distances = self._calculate_path_distances(path_points)
        curvature_limited_velocities = self._calculate_curvature_limited_velocities(x_fine, y_fine)
        
        # 2단계: 웨이포인트 제약과 곡률 제약 통합
        target_velocities = self._combine_waypoint_and_curvature_velocities(
            path_points, wx, wy, curvature_limited_velocities)
        
        # 3단계: 통합 물리 제약 적용 (역방향 + 전방향)
        final_velocities = self._apply_unified_physics_constraints(
            path_points, distances, target_velocities)
        
        self.get_logger().info(f"Unified Physics Velocity Profile: "
                              f"Avg: {np.mean(final_velocities):.2f} m/s, "
                              f"Range: [{np.min(final_velocities):.2f}, {np.max(final_velocities):.2f}] m/s")
        
        return final_velocities
    
    def _calculate_curvature_limited_velocities(self, x_fine, y_fine):
        """곡률 기반 횡방향 가속도 제한 속도 계산"""
        curvatures = self._calculate_curvature(x_fine, y_fine)
        curvature_limited_velocities = []
        
        for curvature in curvatures:
            abs_curvature = abs(curvature)
            
            if abs_curvature < 1e-6:  # 직선 구간
                max_vel = self.MAX_SPEED
            else:
                # v_max = √(a_lateral_max / κ) 
                # 횡방향 가속도: a_lateral = v² × κ
                max_vel = math.sqrt(self.MAX_LATERAL_ACCEL / abs_curvature)
            
            # MIN_SPEED와 MAX_SPEED 범위 적용
            max_vel = np.clip(max_vel, self.MIN_SPEED, self.MAX_SPEED)
            curvature_limited_velocities.append(max_vel)
        
        return curvature_limited_velocities
    
    def _combine_waypoint_and_curvature_velocities(self, path_points, wx, wy, curvature_velocities):
        """웨이포인트 기반 속도와 곡률 제한 속도를 결합"""
        waypoint_velocities = self._map_waypoint_velocities_to_path(path_points, wx, wy)
        
        # 두 제약 조건 중 더 작은 값 선택 (더 안전한 속도)
        combined_velocities = []
        for i in range(len(path_points)):
            wp_vel = waypoint_velocities[i]
            curve_vel = curvature_velocities[i] if i < len(curvature_velocities) else self.MAX_SPEED
            
            # 두 제약 중 더 엄격한 것 선택
            final_vel = min(wp_vel, curve_vel)
            combined_velocities.append(final_vel)
        
        return combined_velocities
    
    def _apply_unified_physics_constraints(self, path_points, distances, target_velocities):
        """통합 물리 제약 적용 - 웨이포인트간 부드러운 속도 전환 + 가속도 제한"""
        velocities = target_velocities.copy()
        
        # 1단계: 웨이포인트 기반 역방향 감속 제약
        velocities = self._apply_waypoint_deceleration_constraints(path_points, distances, velocities)
        
        # 2단계: 전방향 가속 제약 (연속성 보장)
        velocities = self._apply_forward_acceleration_limits(distances, velocities)
        
        # 3단계: 역방향 감속 제약 (최종 검증)
        velocities = self._apply_backward_deceleration_limits(distances, velocities)
        
        # 4단계: 최소 가속도 보장 (정상상태오차 방지)
        velocities = self._ensure_minimum_acceleration(distances, velocities)
        
        return velocities
    
    def _apply_waypoint_deceleration_constraints(self, path_points, distances, velocities):
        """웨이포인트 정지/저속 구간을 위한 감속 제약"""
        # 정지 및 저속 웨이포인트 찾기
        critical_points = []
        for wp_x, wp_y, mission_type, target_speed in self.raw_waypoints:
            if mission_type in [2, 4] or (target_speed >= 0 and target_speed <= 1.0):
                # 가장 가까운 경로점 찾기
                closest_idx = 0
                min_dist = float('inf')
                for i, (px, py) in enumerate(path_points):
                    dist = math.hypot(px - wp_x, py - wp_y)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                
                constraint_speed = 0.0 if mission_type in [2, 4] else target_speed
                critical_points.append((closest_idx, distances[closest_idx], constraint_speed))
        
        # 각 중요 지점에서 역방향 감속 적용
        for target_idx, target_dist, target_speed in critical_points:
            for i in range(target_idx, -1, -1):
                distance_to_target = target_dist - distances[i]
                if distance_to_target >= 0:
                    # 감속 가능한 최대 속도: v = √(v_target² + 2*a_decel*s)
                    max_speed_for_decel = math.sqrt(
                        target_speed**2 + 2 * self.MAX_DECEL * distance_to_target
                    )
                    velocities[i] = min(velocities[i], max_speed_for_decel)
        
        return velocities
    
    def _apply_forward_acceleration_limits(self, distances, velocities):
        """전방향 가속도 제한 - 부드러운 속도 전환"""
        result_velocities = velocities.copy()
        
        for i in range(1, len(velocities)):
            distance_step = distances[i] - distances[i-1]
            prev_speed = result_velocities[i-1]
            target_speed = velocities[i]
            
            if distance_step > 0:
                if target_speed > prev_speed:  # 가속 구간
                    # 최대 가속도 제한: v = √(v₀² + 2*a_max*s)
                    max_achievable_speed = math.sqrt(
                        prev_speed**2 + 2 * self.MAX_ACCEL * distance_step
                    )
                    result_velocities[i] = min(target_speed, max_achievable_speed)
                else:  # 감속 구간
                    # 부드러운 감속 적용
                    result_velocities[i] = target_speed
        
        return result_velocities
    
    def _apply_backward_deceleration_limits(self, distances, velocities):
        """역방향 감속도 제한 - 최종 검증"""
        result_velocities = velocities.copy()
        
        for i in range(len(velocities) - 2, -1, -1):
            distance_step = distances[i+1] - distances[i]
            next_speed = result_velocities[i+1]
            current_speed = result_velocities[i]
            
            if distance_step > 0 and current_speed > next_speed:
                # 감속 가능한 최대 속도: v = √(v_next² + 2*a_decel*s)
                max_speed_for_decel = math.sqrt(
                    next_speed**2 + 2 * self.MAX_DECEL * distance_step
                )
                result_velocities[i] = min(current_speed, max_speed_for_decel)
        
        return result_velocities
    
    def _ensure_minimum_acceleration(self, distances, velocities):
        """최소 가속도 보장 - 정상상태오차 방지"""
        result_velocities = velocities.copy()
        
        # 너무 작은 속도 변화는 최소 가속도로 보정
        for i in range(1, len(velocities)):
            distance_step = distances[i] - distances[i-1]
            prev_speed = result_velocities[i-1]
            current_speed = result_velocities[i]
            
            if distance_step > 0.1:  # 10cm 이상 거리에서만 적용
                speed_diff = current_speed - prev_speed
                if abs(speed_diff) > 0 and abs(speed_diff) / distance_step < self.MIN_ACCEL:
                    # 최소 가속도 적용
                    if speed_diff > 0:  # 가속
                        result_velocities[i] = prev_speed + self.MIN_ACCEL * distance_step
                    else:  # 감속
                        result_velocities[i] = prev_speed - self.MIN_ACCEL * distance_step
                    
                    # 범위 제한
                    result_velocities[i] = np.clip(result_velocities[i], 0.0, self.MAX_SPEED)
        
        return result_velocities
    
    def _map_waypoint_velocities_to_path(self, path_points, wx, wy):
        """웨이포인트별 목표 속도를 경로점에 매핑"""
        velocities = []
        
        for i, (px, py) in enumerate(path_points):
            # 가장 가까운 웨이포인트 찾기
            closest_wp_idx = 0
            min_dist = float('inf')
            
            for j, (wpx, wpy, _, target_speed) in enumerate(self.raw_waypoints):
                dist = math.hypot(px - wpx, py - wpy)
                if dist < min_dist:
                    min_dist = dist
                    closest_wp_idx = j
            
            # 목표 속도 결정
            _, _, mission_type, target_speed = self.raw_waypoints[closest_wp_idx]
            
            if target_speed < 0:  # -1.0은 기본 최대 속도 사용
                target_vel = self.MAX_SPEED
            else:
                # 지정된 목표 속도 사용 (min/max 범위 적용)
                target_vel = np.clip(target_speed, self.MIN_SPEED, self.MAX_SPEED)
            
            velocities.append(target_vel)
        
        return velocities



    def _calculate_path_distances(self, path_points):
        """경로를 따른 누적 거리 계산"""
        distances = [0.0]
        for i in range(1, len(path_points)):
            dist = math.hypot(
                path_points[i][0] - path_points[i-1][0],
                path_points[i][1] - path_points[i-1][1]
            )
            distances.append(distances[-1] + dist)
        return distances

    def _calculate_curvature(self, x, y):
        """경로의 곡률 계산"""
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # 곡률 공식: κ = |dx*ddy - dy*ddx| / (dx²+dy²)^(3/2)
        curvature = (dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
        return curvature

    # === 시각화 ===
    
    def _publish_path_visualization(self):
        """경로 시각화"""
        # 경로 퍼블리시
        path_msg = Path()
        path_msg.header.frame_id = self.MAP_FRAME
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for p in self.full_path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = p[0], p[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        
        # 웨이포인트 마커
        if self.current_waypoint_idx == 0:
            self._publish_waypoint_markers()
        
        # 속도 마커
        self._publish_speed_markers()

    def _publish_waypoint_markers(self):
        """웨이포인트 마커 퍼블리시"""
        marker_array = MarkerArray()
        
        for i, (x, y, mission, target_speed) in enumerate(self.raw_waypoints):
            marker = Marker()
            marker.header.frame_id = self.MAP_FRAME
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.5
            
            # 미션 타입별 색상
            if mission == 2:  # 드론 이륙 대기 지점
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # 노란색
            elif mission == 4:  # 종료 지점
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # 빨간색
            elif target_speed > 0 and target_speed <= 1.0:  # 저속 구간
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0  # 주황색
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0  # 파란색
                
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        self.waypoint_marker_pub.publish(marker_array)

    def _publish_speed_markers(self):
        """속도를 반투명 기둥과 텍스트로 시각화"""
        marker_array = MarkerArray()
        
        # 2m 간격으로 표시 (너무 많으면 시각적으로 복잡)
        step = max(1, int(0.5 / self.PATH_DENSITY))
        
        for i in range(0, len(self.full_path_points), step):
            if i >= len(self.full_target_velocities):
                break
                
            x, y = self.full_path_points[i]
            vel = self.full_target_velocities[i]
            
            # 속도 기둥 (반투명 실린더)
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = self.MAP_FRAME
            cylinder_marker.header.stamp = self.get_clock().now().to_msg()
            cylinder_marker.ns = "speed_cylinders"
            cylinder_marker.id = i
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            
            # 기둥 위치 (바닥에서 시작)
            cylinder_height = max(0.2, vel * 0.5)  # 속도에 비례한 높이 (최소 0.2m)
            cylinder_marker.pose.position.x = x
            cylinder_marker.pose.position.y = y
            cylinder_marker.pose.position.z = cylinder_height / 2.0  # 중심점이 높이의 절반
            cylinder_marker.pose.orientation.w = 1.0
            
            # 기둥 크기
            cylinder_marker.scale.x = 0.15  # 지름
            cylinder_marker.scale.y = 0.15  # 지름
            cylinder_marker.scale.z = cylinder_height  # 높이
            
            # 속도에 따른 색상 (파란색 → 초록색 → 빨간색)
            if vel <= 1.0:
                # 저속: 파란색
                cylinder_marker.color.r = 0.0
                cylinder_marker.color.g = 0.5
                cylinder_marker.color.b = 1.0
            elif vel <= 2.0:
                # 중속: 초록색
                cylinder_marker.color.r = 0.0
                cylinder_marker.color.g = 1.0
                cylinder_marker.color.b = 0.0
            else:
                # 고속: 빨간색
                cylinder_marker.color.r = 1.0
                cylinder_marker.color.g = 0.0
                cylinder_marker.color.b = 0.0
            
            cylinder_marker.color.a = 0.6  # 반투명
            
            marker_array.markers.append(cylinder_marker)
            
            # 속도 텍스트 (기둥 위에)
            text_marker = Marker()
            text_marker.header.frame_id = self.MAP_FRAME
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "speed_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 텍스트 위치 (기둥 위)
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = cylinder_height + 0.3  # 기둥 위 0.3m
            text_marker.pose.orientation.w = 1.0
            
            # 텍스트 내용 및 스타일
            text_marker.text = f"{vel:.1f}m/s"
            text_marker.scale.z = 0.3  # 텍스트 크기
            
            # 흰색 텍스트 (가독성 향상)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        self.speed_marker_pub.publish(marker_array)

    def _publish_lookahead_marker(self, goal_point):
        """전방주시점 마커 퍼블리시"""
        marker = Marker()
        marker.header.frame_id = self.MAP_FRAME
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal_point[0], goal_point[1], 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.8
        
        self.lookahead_marker_pub.publish(marker)

    # === 유틸리티 함수 ===
    
    def _load_waypoints(self):
        """웨이포인트 데이터 로딩"""
        return [
            (-124.48, 43.38, 1, -1.0),
            (-132.71, 58.04, 1, 2.0),    
            (-132.87, 64.00, 2, 0.0),    # 드론 이륙 대기
            (-129.23, 69.36, 3, -1.0),    
            (-120.85, 73.20, 3, -1.0),    
            (-117.45, 73.15, 3, -1.0),    
            (-113.63, 72.64, 3, -1.0),    
            (-104.97, 77.01, 3, -1.0),    
            (-94.75, 84.41, 3, -1.0),     
            (-91.71, 86.98, 3, -1.0),     
            (-80.82, 97.95, 3, -1.0),     
            (-76.74, 99.61, 3, 1.0),     # 복잡한 구간 시작
            (-73.90, 98.63, 3, 1.0),     # 1.0m/s 구간
            (-72.13, 98.65, 3, 1.0),     # 1.0m/s 구간  
            (-62.96, 99.09, 4, 0.0)      # 종료점
        ]

    def _quat_to_euler(self, q):
        """쿼터니언을 오일러 각으로 변환 (Yaw만)"""
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return 0, 0, math.atan2(t3, t4)

    def _stop_vehicle(self):
        """차량 정지"""
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
