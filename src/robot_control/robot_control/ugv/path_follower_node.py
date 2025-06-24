#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
from mission_admin_interfaces.srv import MissionComplete

import tf2_ros
from tf2_ros import TransformException

import math
import numpy as np
import threading
import sys
import os

from robot_control.utils.waypoint_parser import load_waypoints_from_csv, WaypointData
from robot_control.ugv.path_planner import PathPlanner
from robot_control.ugv.velocity_profiler import VelocityProfiler
from robot_control.ugv.pure_pursuit import PurePursuitController
from robot_control.utils import viz_factory

class PathFollowerNode(Node):
    """
    S-Curve 속도 프로파일 기반 경로 추종 노드
    - 드론 탑재 여부에 따라 저크(Jerk)와 가속도를 동적으로 제어하여 최적화된 주행 구현
    - 경로 완료 시 미션 컨트롤에 신호 전송 (경로의 의미는 해석하지 않음)
    """
    def __init__(self):
        super().__init__('path_follower_node')

        # === 핵심 파라미터 ===
        self.declare_parameter('max_jerk_with_drone', 1.0)
        self.declare_parameter('max_jerk_default', 3.0)
        self.declare_parameter('max_accel_with_drone', 0.5)
        self.declare_parameter('max_accel_default', 2.0)

        self.declare_parameter('lookahead_k', 2.0)
        self.declare_parameter('lookahead_min', 0.5)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('max_speed', 6.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_decel', 1.0)
        self.declare_parameter('max_lateral_accel', 2.0)
        self.declare_parameter('waypoint_reach_threshold', 1.5)
        self.declare_parameter('path_density', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('vehicle_base_frame', 'X1_asp')
        
        # === 경로 로딩 파라미터 ===
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('use_mission_ids', True)

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
        self.is_orienting = False
        self.last_closest_idx = 0

        # === 리팩토링된 모듈 초기화 ===
        self.path_planner = PathPlanner(path_density=self.PATH_DENSITY)
        self.velocity_profiler = VelocityProfiler(
            max_speed=self.MAX_SPEED,
            min_speed=self.MIN_SPEED,
            max_accel=self.current_max_accel,
            max_decel=self.MAX_DECEL,
            max_jerk=self.current_max_jerk,
            max_lateral_accel=self.MAX_LATERAL_ACCEL
        )
        self.pursuit_controller = PurePursuitController(
            lookahead_k=self.LOOKAHEAD_K,
            lookahead_min=self.LOOKAHEAD_MIN,
            lookahead_max=self.LOOKAHEAD_MAX
        )

        # 경로 데이터
        self.raw_waypoints = self._load_waypoints()
        if not self.raw_waypoints:
            self.get_logger().error("웨이포인트 로드 실패. 노드 종료")
            raise Exception("웨이포인트 로드 실패")
            
        self.main_path_points = self.path_planner.generate_path_from_waypoints(self.raw_waypoints)
        self.full_path_points = []
        self.full_target_velocities = []

        # === 미션 완료 신호 매핑 ===
        self.waypoint_mission_mapping = {
            2: 1,
            len(self.raw_waypoints) - 1: 3
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
        
        self.mission_complete_client = self.create_client(MissionComplete, '/mission_complete')
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.path_update_timer = self.create_timer(0.2, self.update_full_path_and_velocity)
        self.state_timer = self.create_timer(0.1, self.publish_state)
        self.input_thread = threading.Thread(target=self._command_input_loop, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info("S-Curve Path Follower v4.1 초기화 완료")
        self.get_logger().info(f"   - 웨이포인트 파일: {self.waypoint_file if self.waypoint_file else '기본 경로'}")
        self.get_logger().info(f"   - 로드된 웨이포인트: {len(self.raw_waypoints)}개")
        self.get_logger().info("   - 현재 모드: [With Drone]. 'go' 명령 대기 중")
        self.get_logger().info("=" * 60)
        self.get_logger().info("미션을 시작하려면 터미널에서 'go'를 입력하고 Enter를 누르세요!")
        self.get_logger().info("=" * 60)

    def odom_callback(self, msg: Odometry):
        self.current_speed = msg.twist.twist.linear.x

    def mission_command_callback(self, msg: String):
        command = msg.data.lower().strip()
        self.get_logger().info(f"대시보드 명령 수신: '{command}'")
        self._process_command(command)

    def _process_command(self, cmd: str):
        if not cmd:
            return
            
        self.get_logger().info(f"명령 처리 중: '{cmd}'")
        
        if self.is_waiting_for_go and cmd == 'go':
            self.is_waiting_for_go = False
            self.path_update_timer.cancel()
            self.get_logger().info("GO 명령 수신! 미션을 시작")
        elif self.is_mission_paused and cmd == 'resume':
            self._resume_mission()
        elif cmd == 'stop':
            self._stop_vehicle()
            self.is_mission_paused = True
            self.get_logger().info("STOP 명령 수신! 미션을 일시 중지")
        else:
            current_state = "WAITING_FOR_GO" if self.is_waiting_for_go else \
                           "PAUSED" if self.is_mission_paused else "ACTIVE"
            self.get_logger().warn(f"현재 상태({current_state})에서 유효하지 않은 명령: '{cmd}'")

    def publish_state(self):
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
        if not self.use_mission_ids:
            self.get_logger().info(f"미션 완료 신호 전송 비활성화됨 (ID: {mission_id})")
            return
            
        if not self.mission_complete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"미션 컨트롤 서비스를 찾을 수 없음 (ID: {mission_id}) - 서비스 없이 계속 진행")
            return
            
        request = MissionComplete.Request()
        request.mission_id = mission_id
        
        try:
            future = self.mission_complete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"미션 완료 신호 전송 성공 (ID: {mission_id})")
                else:
                    self.get_logger().warn(f"미션 완료 신호 거부됨 (ID: {mission_id}) - 계속 진행")
            else:
                self.get_logger().warn(f"미션 완료 신호 전송 타임아웃 (ID: {mission_id}) - 계속 진행")
        except Exception as e:
            self.get_logger().warn(f"미션 완료 신호 전송 실패 (ID: {mission_id}): {e} - 계속 진행")

    def _update_vehicle_pose(self):
        try:
            full_vehicle_frame_id = f"{self.VEHICLE_BASE_FRAME}/base_link"
            trans = self.tf_buffer.lookup_transform(
                self.MAP_FRAME, full_vehicle_frame_id, rclpy.time.Time())
            pos = trans.transform.translation
            quat = trans.transform.rotation
            _, _, yaw = self._quat_to_euler(quat)
            self.vehicle_pose_map = (pos.x, pos.y, yaw)
            return True
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {full_vehicle_frame_id} to {self.MAP_FRAME}: {ex}', throttle_duration_sec=1.0)
            return False

    def control_loop(self):
        if not self._update_vehicle_pose():
            self._stop_vehicle()
            return

        if self.is_orienting:
            if self._orient_vehicle_for_next_segment():
                self.is_orienting = False
                self.get_logger().info("Vehicle oriented. Continuing mission")
            return

        # 1. 루프 시작 시 미션 상태를 먼저 확인
        if self.is_waiting_for_go or self.is_mission_complete or self.is_mission_paused:
            self._stop_vehicle()
            return
            
        # 2. 웨이포인트 도달 여부를 확인하여 미션 상태를 변경
        self._check_waypoint_arrival()
        
        # 3. (가장 중요) 상태 변경 직후, 다시 한 번 미션 상태를 확인하여 즉시 정지
        #    이것이 비정상적인 Twist 명령이 나가는 것을 막는 핵심 코드입니다.
        if self.is_mission_paused or self.is_mission_complete:
            self._stop_vehicle()
            return

        # 4. 위 모든 조건에 해당하지 않을 때만 경로 추종 제어 실행
        current_x, current_y, current_yaw = self.vehicle_pose_map
        
        cmd_vel, goal_point = self.pursuit_controller.calculate_control_command(
            (current_x, current_y), current_yaw, self.current_speed,
            self.full_path_points, self.full_target_velocities, [self.last_closest_idx]
        )
        
        if goal_point is None:
            self.get_logger().warn("목표 지점을 찾을 수 없어 정지합니다.", throttle_duration_sec=2.0)
            self._stop_vehicle()
            return
        
        self.cmd_vel_pub.publish(cmd_vel)
        self._publish_lookahead_marker(goal_point)

    def _command_input_loop(self):
        while rclpy.ok():
            try:
                cmd = input(">>> ").strip().lower()
                
                if cmd:
                     self._process_command(cmd)
                
            except (KeyboardInterrupt, EOFError):
                self.get_logger().info("키보드 입력 스레드 종료됨.")
                break
            except Exception as e:
                self.get_logger().warn(f"입력 처리 중 오류 발생: {e}")
                import time
                time.sleep(1)

    def _resume_mission(self):
        self.get_logger().info("Switching to high-performance mode (Drone departed).")
        self.current_max_accel = self.MAX_ACCEL_DEFAULT
        self.current_max_jerk = self.MAX_JERK_DEFAULT
        self.velocity_profiler.update_dynamic_limits(self.current_max_accel, self.current_max_jerk)

        remaining_waypoints = self.raw_waypoints[self.current_waypoint_idx:]
        if not remaining_waypoints:
            self.is_mission_complete = True
            self.get_logger().info("No more waypoints. Mission complete.")
            return
            
        self.main_path_points = self.path_planner.generate_path_from_waypoints(remaining_waypoints)
        
        # <<< 수정됨 >>> 경로가 재생성되므로, 경로 탐색 시작 인덱스를 반드시 0으로 초기화해야 합니다.
        self.last_closest_idx = 0
        
        self.update_full_path_and_velocity()
        
        self.is_mission_paused = False
        self.is_orienting = True

    def _orient_vehicle_for_next_segment(self):
        closest_idx = self.pursuit_controller.find_closest_point_idx(
            (self.vehicle_pose_map[0], self.vehicle_pose_map[1]), self.full_path_points, [self.last_closest_idx]
        )
        if closest_idx >= len(self.full_path_points) - 1: 
            return True
        
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

    def _publish_path_visualization(self):
        """viz_factory를 사용하여 경로 관련 시각화 데이터를 생성하고 발행."""
        if not self.full_path_points: return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.MAP_FRAME

        # 1. Path 메시지 생성 및 발행
        path_msg = viz_factory.create_ugv_path_marker(header, self.full_path_points)
        self.path_pub.publish(path_msg)

        # 2. 웨이포인트 마커 생성 및 발행
        waypoint_markers = viz_factory.create_ugv_waypoint_markers(header, self.raw_waypoints)
        self.waypoint_marker_pub.publish(waypoint_markers)

        # 3. 속도 마커 생성 및 발행
        speed_markers = viz_factory.create_speed_markers(
            header, self.full_path_points, self.full_target_velocities, self.PATH_DENSITY, self.MAX_SPEED
        )
        self.speed_marker_pub.publish(speed_markers)

    def _publish_lookahead_marker(self, goal_point):
        """viz_factory를 사용하여 Lookahead 마커를 생성하고 발행."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.MAP_FRAME
        
        marker = viz_factory.create_lookahead_marker(header, goal_point)
        self.lookahead_marker_pub.publish(marker)

    def _load_waypoints(self):
        if not self.waypoint_file:
            self.get_logger().error("웨이포인트 파일이 지정되지 않음")
            return None
        if not os.path.exists(self.waypoint_file):
            self.get_logger().error(f"웨이포인트 파일을 찾을 수 없음: {self.waypoint_file}")
            return None
        try:
            waypoints = self._load_waypoints_from_csv(self.waypoint_file)
            self.get_logger().info(f"웨이포인트 파일 로드 성공: {self.waypoint_file}")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 로드 실패: {e}")
            return None

    def _load_waypoints_from_csv(self, file_path):
        try:
            waypoint_data_list = load_waypoints_from_csv(file_path)
            waypoints = [wp_data.to_tuple() for wp_data in waypoint_data_list]
            self.get_logger().info(f"CSV 파서를 사용하여 {len(waypoints)}개 웨이포인트 로드 완료")
            return waypoints
        except Exception as e:
            self.get_logger().error(f"CSV 파싱 오류: {e}")
            raise

    def _quat_to_euler(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return 0, 0, math.atan2(t3, t4)

    def _stop_vehicle(self):
        self.cmd_vel_pub.publish(Twist())

    def update_full_path_and_velocity(self):
        if self.is_waiting_for_go and not self._update_vehicle_pose(): return
        
        current_pose = self.vehicle_pose_map if self.vehicle_pose_map else (0,0,0)
        start_x, start_y, _ = current_pose

        if not self.main_path_points:
            self.get_logger().warn("Main path is empty, cannot generate full path.")
            return

        goal_x, goal_y = self.main_path_points[0]
        initial_path = self.path_planner._generate_straight_path((start_x, start_y), (goal_x, goal_y))
        
        self.full_path_points = initial_path + self.main_path_points
        
        final_waypoint = self.raw_waypoints[-1]
        end_vel = 0.0 if len(final_waypoint) > 2 and final_waypoint[2] == 4 else 0.0
        
        distances = self.path_planner.calculate_path_distances(self.full_path_points)
        curvatures = self.path_planner.calculate_curvature(self.full_path_points)
        self.full_target_velocities = self.velocity_profiler.generate_scurve_profile(
            self.full_path_points, distances, curvatures, self.raw_waypoints, 
            start_vel=self.current_speed, end_vel=end_vel
        )
        
        self._publish_path_visualization()

    def _check_waypoint_arrival(self):
        """웨이포인트 도달 확인 및 상태 업데이트"""
        if self.current_waypoint_idx >= len(self.raw_waypoints):
            return
            
        waypoint = self.raw_waypoints[self.current_waypoint_idx]
        wp_x, wp_y = waypoint[0], waypoint[1]
        
        if self.vehicle_pose_map is None:
            return
            
        dist = math.hypot(self.vehicle_pose_map[0] - wp_x, self.vehicle_pose_map[1] - wp_y)
        
        if dist < self.REACH_THRESHOLD:
            self.get_logger().info(f"웨이포인트 {self.current_waypoint_idx} 도달 (거리: {dist:.2f}m)")
            
            if len(waypoint) > 2:
                mission_type = waypoint[2]
                if mission_type == 2:
                    self.is_mission_paused = True
                    self.get_logger().info("드론 이륙 지점 도달. 차량을 일시 정지합니다.")
                elif mission_type == 4:
                    self.is_mission_complete = True
                    self.get_logger().info("최종 목적지 도달. 미션을 완료합니다!")
            
            if self.current_waypoint_idx in self.waypoint_mission_mapping:
                mission_id = self.waypoint_mission_mapping[self.current_waypoint_idx]
                self.send_mission_complete(mission_id)
            
            # 다음 웨이포인트 인덱스로 업데이트
            self.current_waypoint_idx += 1
            
            # (방어 코드) 다음 웨이포인트로 넘어가기 직전, last_closest_idx를 현재 위치 기준으로 갱신합니다.
            # 이는 혹시라도 제어 루프가 한 번 더 실행될 경우, 뒤를 돌아보는 현상을 방지합니다.
            if self.full_path_points and self.vehicle_pose_map:
                # last_closest_idx는 리스트로 전달되어 내부에서 값이 변경됩니다.
                last_closest_idx_ref = [self.last_closest_idx]
                self.pursuit_controller.find_closest_point_idx(
                    (self.vehicle_pose_map[0], self.vehicle_pose_map[1]),
                    self.full_path_points,
                    last_closest_idx_ref
                )
                self.last_closest_idx = last_closest_idx_ref[0]

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathFollowerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, Exception) as e:
        if 'node' in locals():
            node.get_logger().info(f"Shutting down due to {e}...")
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()