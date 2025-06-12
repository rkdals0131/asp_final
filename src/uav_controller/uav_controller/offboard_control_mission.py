#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 및 서비스 타입 임포트 ---
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import SetBool # 간단한 시작/정지용 서비스 (True: 시작)
from tf2_geometry_msgs import do_transform_point

# --- TF2 관련 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import csv
import math

class MissionPlanner(Node):
    """
    미션 계획 노드. 서비스 호출을 받아 지정된 웨이포인트를 순회하는 미션을 수행합니다.
    map 좌표계의 웨이포인트를 드론의 local 좌표계로 변환하여 제어합니다.
    """

    def __init__(self):
        super().__init__('offboard_control_mission')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publisher ---
        self.command_pose_publisher = self.create_publisher(PoseStamped, "/command/pose", 10)

        # --- Subscriber ---
        self.odometry_subscriber = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile
        )

        # --- Service Server ---
        self.mission_service = self.create_service(SetBool, "~/start_mission", self.start_mission_callback)

        # --- TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 미션 관련 변수 ---
        self.mission_waypoints_map = []
        self.current_waypoint_index = 0
        self.state = "IDLE"
        self.current_odom = None # 드론의 현재 위치 (odom 프레임)
        self.safe_altitude = 30.0
        
        self.load_waypoints_from_csv('uav_wp.csv')

        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("Mission Planner is ready. Waiting for a service call to start.")

    def load_waypoints_from_csv(self, file_path):
        """CSV 파일에서 웨이포인트를 읽어옵니다."""
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) == 3:
                        self.mission_waypoints_map.append([float(p) for p in row])
            self.get_logger().info(f"Successfully loaded {len(self.mission_waypoints_map)} waypoints from {file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found at {file_path}")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")

    def odometry_callback(self, msg: VehicleOdometry):
        """드론의 현재 위치 및 자세 정보를 업데이트합니다."""
        self.current_odom = msg

    def start_mission_callback(self, request: SetBool.Request, response: SetBool.Response):
        """미션 시작 서비스 콜백."""
        if request.data:
            if self.state == "IDLE":
                self.get_logger().info("Mission start command received. Changing state to TAKEOFF.")
                self.state = "TAKEOFF"
                response.success = True
                response.message = "Mission started."
            else:
                response.success = False
                response.message = f"Cannot start mission, current state is {self.state}"
        else:
            self.state = "IDLE"
            response.success = True
            response.message = "Mission stopped and returned to IDLE."
            
        return response

    def transform_map_to_odom(self, map_point_list):
        """
        map 좌표계의 포인트를 odom 좌표계로 변환합니다.
        TF2를 사용하여 실제 좌표 변환을 수행합니다.
        """
        try:
            # 'map'에서 'odom'으로의 변환 정보를 조회합니다.
            # 이 정보는 C++ 노드가 방송해주는 TF 덕분에 가능합니다.
            transform_stamped = self.tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            
            point_in_map = Point()
            point_in_map.x = map_point_list[0]
            point_in_map.y = map_point_list[1]
            point_in_map.z = map_point_list[2]

            # 실제 좌표 변환 수행
            point_in_odom = do_transform_point(point_in_map, transform_stamped)
            
            return [point_in_odom.x, point_in_odom.y, point_in_odom.z]

        except TransformException as ex:
            self.get_logger().error(f'Could not transform map to odom: {ex}')
            return None

    def main_loop(self):
        """주기적으로 실행되는 메인 상태 머신."""
        if self.state == "IDLE":
            return

        if self.current_odom is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        if self.state == "TAKEOFF":
            # 이륙 로직: 현재 위치에서 5m 상공으로 이동
            takeoff_pose = PoseStamped()
            takeoff_pose.header.stamp = self.get_clock().now().to_msg()
            takeoff_pose.header.frame_id = 'odom'
            takeoff_pose.pose.position.x = self.current_odom.position[0]
            takeoff_pose.pose.position.y = self.current_odom.position[1]
            takeoff_pose.pose.position.z = -5.0 # NED 좌표계 기준
            self.command_pose_publisher.publish(takeoff_pose)
            
            # 이륙 완료 확인 (예: 고도가 -4.8m 이상 도달 시)
            if self.current_odom.position[2] < -4.8:
                self.get_logger().info("Takeoff complete. Moving to the first safe waypoint.")
                self.state = "MOVE_TO_SAFE_WAYPOINT"

        elif self.state == "MOVE_TO_SAFE_WAYPOINT":
            if self.current_waypoint_index >= len(self.mission_waypoints_map):
                self.state = "MISSION_COMPLETE"
                return

            target_marker_map = self.mission_waypoints_map[self.current_waypoint_index]
            safe_waypoint_map = [target_marker_map[0], target_marker_map[1], -self.safe_altitude] # NED
            
            target_pose_odom_list = self.transform_map_to_odom(safe_waypoint_map)

            if target_pose_odom_list:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'odom'
                pose_msg.pose.position.x = float(target_pose_odom_list[0])
                pose_msg.pose.position.y = float(target_pose_odom_list[1])
                pose_msg.pose.position.z = float(target_pose_odom_list[2])
                self.command_pose_publisher.publish(pose_msg)
            
            # TODO: 해당 웨이포인트에 도착했는지 확인하는 로직 필요
            # if is_arrived:
            #   self.state = "DESCEND_TO_MARKER"

        # ... (이하 다른 상태들) ...

        elif self.state == "MISSION_COMPLETE":
            self.get_logger().info("All waypoints visited. Mission complete.")
            self.state = "IDLE"


def main(args=None):
    rclpy.init(args=args)
    mission_planner = MissionPlanner()
    rclpy.spin(mission_planner)
    mission_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
