#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray

# --- TF2 관련 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import os
import datetime
import math

class UAVDashboard(Node):
    """
    드론과 차량의 시스템 상태, 임무 정보, 원격측정 데이터를
    하나의 터미널에 실시간으로 표시하는 통합 대시보드 노드.
    """

    def __init__(self):
        super().__init__('uav_dashboard_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # --- ANSI 색상 코드 ---
        self.COLOR_GREEN = '\033[92m'
        self.COLOR_RED = '\033[91m'
        self.COLOR_YELLOW = '\033[93m'
        self.COLOR_CYAN = '\033[96m'
        self.COLOR_BOLD = '\033[1m'
        self.COLOR_END = '\033[0m'

        # --- 데이터 저장을 위한 멤버 변수 ---
        self.drone_local_pos = None
        self.vehicle_odom = None
        self.marker_detections = None
        self.drone_state = "INITIALIZING"
        self.vehicle_state = "INITIALIZING"

        # --- Pre-flight Check를 위한 변수 ---
        self.check_timeout = 2.0  # 2초 이내에 메시지가 없으면 NO-GO
        self.topic_last_seen = {
            'PX4_LOC_POS': 0.0,
            'VEHICLE_ODOM': 0.0,
            'CAMERA_IMG': 0.0,
        }
        self.tf_status = False

        # TF 프레임 이름
        self.drone_frame_id = "x500_gimbal_0/base_link"
        self.vehicle_frame_id = "X1_asp/base_link"
        self.map_frame = "map"

        # TF2 Listener 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- QoS 프로파일 정의 ---
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10 # 상태 메시지는 약간의 버퍼를 둠
        )

        # --- Subscriber 초기화 ---
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, "/model/X1/odometry", self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(Image, "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image", self.camera_image_callback, qos_best_effort)
        self.create_subscription(Detection3DArray, "/marker_detections", self.marker_detection_callback, qos_reliable)
        self.create_subscription(String, "/drone/state", self.drone_state_callback, qos_reliable)
        self.create_subscription(String, "/vehicle/state", self.vehicle_state_callback, qos_reliable)

        # 30Hz 업데이트 타이머
        self.timer = self.create_timer(1.0 / 30.0, self.update_dashboard)
        
        self.get_logger().info("UAV Telemetry Dashboard v2 is running.")

    # --- 콜백 함수들 ---
    def get_current_time_sec(self):
        """현재 노드 시간(시뮬레이션 시간)을 초 단위로 반환"""
        return self.get_clock().now().nanoseconds / 1e9

    def drone_local_pos_callback(self, msg: VehicleLocalPosition):
        self.drone_local_pos = msg
        self.topic_last_seen['PX4_LOC_POS'] = self.get_current_time_sec()

    def vehicle_odometry_callback(self, msg: Odometry):
        self.vehicle_odom = msg
        self.topic_last_seen['VEHICLE_ODOM'] = self.get_current_time_sec()
        
    def camera_image_callback(self, msg: Image):
        self.topic_last_seen['CAMERA_IMG'] = self.get_current_time_sec()

    def marker_detection_callback(self, msg: Detection3DArray):
        self.marker_detections = msg

    def drone_state_callback(self, msg: String):
        self.drone_state = msg.data

    def vehicle_state_callback(self, msg: String):
        self.vehicle_state = msg.data

    # --- 데이터 처리 및 포맷팅 ---
    def ned_to_gps(self, x, y, z, ref_lat, ref_lon, ref_alt):
        """NED 좌표를 GPS 좌표로 변환합니다."""
        if ref_lat is None or ref_lon is None or ref_alt is None:
            return None, None, None
            
        earth_radius = 6378137.0  # WGS84 타원체의 적도 반지름
        
        d_lat = math.degrees(y / earth_radius)
        d_lon = math.degrees(x / (earth_radius * math.cos(math.radians(ref_lat))))
        
        new_lat = ref_lat + d_lat
        new_lon = ref_lon + d_lon
        new_alt = ref_alt - z # NED의 Z축(Down)은 고도(Up)와 반대
        
        return new_lat, new_lon, new_alt

    def format_status(self, label, is_ok):
        """상태 라벨을 포맷팅합니다 (GO/NO-GO)."""
        status_str = f"{self.COLOR_GREEN}GO  {self.COLOR_END}" if is_ok else f"{self.COLOR_RED}NO-GO{self.COLOR_END}"
        return f"  {label:<18}: [{status_str}]"
    
    def format_data(self, label, value, unit=""):
        """데이터 라벨과 값을 포맷팅합니다."""
        return f"  {label:<18}: {self.COLOR_CYAN}{value}{self.COLOR_END} {unit}"

    # --- 메인 업데이트 루프 ---
    def update_dashboard(self):
        """화면을 지우고 최신 정보로 대시보드를 다시 그립니다."""
        # 1. TF 정보 업데이트
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.drone_frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            self.tf_status = True
        except TransformException:
            self.tf_status = False

        # 2. Pre-flight Check
        now = self.get_current_time_sec()
        px4_ok = (now - self.topic_last_seen['PX4_LOC_POS']) < self.check_timeout
        vehicle_odom_ok = (now - self.topic_last_seen['VEHICLE_ODOM']) < self.check_timeout
        camera_ok = (now - self.topic_last_seen['CAMERA_IMG']) < self.check_timeout
        all_systems_go = all([px4_ok, vehicle_odom_ok, camera_ok, self.tf_status])

        # 3. 화면 클리어 및 대시보드 그리기
        os.system('clear')
        
        # === 섹션 1: 시스템 상태 (Pre-flight Check) ========================
        status_header = "SYSTEMS STATUS (PRE-FLIGHT CHECK)"
        all_go_str = f"({self.COLOR_GREEN}ALL SYSTEMS GO{self.COLOR_END})" if all_systems_go else f"({self.COLOR_RED}SYSTEMS NOT READY{self.COLOR_END})"
        print(f"{self.COLOR_BOLD}{'='*70}\n{status_header:^70}\n{all_go_str:^80}\n{'='*70}{self.COLOR_END}")
        print(self.format_status("PX4 (Position)", px4_ok))
        print(self.format_status("Vehicle (Odom)", vehicle_odom_ok))
        print(self.format_status("Gimbal Camera", camera_ok))
        print(self.format_status("TF (Coordinates)", self.tf_status))

        # === 섹션 2: 임무 정보 (Marker Detections) ==========================
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'MISSION PAYLOAD (MARKER DETECTIONS)':^70}\n{'-'*70}{self.COLOR_END}")
        if self.marker_detections and self.drone_local_pos:
            if not self.marker_detections.detections:
                print(f"  {self.COLOR_YELLOW}Searching for markers...{self.COLOR_END}")
            else:
                # Local Position 메시지에서 기준 GPS 좌표를 가져옴
                ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
                
                for det in self.marker_detections.detections:
                    marker_id = det.results[0].hypothesis.class_id
                    pos = det.results[0].pose.pose.position
                    
                    # 월드좌표(map 기준)를 NED 좌표계로 간주하여 GPS 계산
                    # (map 원점과 local position 원점이 동일하다고 가정)
                    m_lat, m_lon, m_alt = self.ned_to_gps(pos.x, pos.y, pos.z, ref_lat, ref_lon, ref_alt)
                    
                    print(f"  {self.COLOR_BOLD}MARKER ID: {marker_id}{self.COLOR_END}")
                    print(f"    - WORLD POS (m) : {self.COLOR_CYAN}X:{pos.x:6.2f} Y:{pos.y:6.2f} Z:{pos.z:6.2f}{self.COLOR_END}")
                    if m_lat:
                        print(f"    - GPS EST (Lat/Lon/Alt) : {self.COLOR_CYAN}{m_lat:.6f}, {m_lon:.6f}, {m_alt:.2f}{self.COLOR_END}")
                    else:
                        print(f"    - GPS EST         : {self.COLOR_YELLOW}Waiting for drone's GPS reference...{self.COLOR_END}")

        else:
            print(f"  {self.COLOR_YELLOW}Waiting for detection data...{self.COLOR_END}")
            
        # === 섹션 3: 플랫폼 원격측정 (Telemetry) =========================
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'PLATFORM TELEMETRY':^70}\n{'-'*70}{self.COLOR_END}")
        
        # --- 드론 정보 ---
        print(f"  {self.COLOR_BOLD}DRONE (x500_gimbal_0){self.COLOR_END}")
        print(self.format_data("STATE", self.drone_state))
        if self.drone_local_pos:
            d_pos = self.drone_local_pos
            d_vel = d_pos.vx, d_pos.vy, d_pos.vz
            d_lat, d_lon, d_alt = self.ned_to_gps(d_pos.x, d_pos.y, d_pos.z, d_pos.ref_lat, d_pos.ref_lon, d_pos.ref_alt)

            print(self.format_data("LOCAL POS (m)", f"X:{d_pos.x:6.2f} Y:{d_pos.y:6.2f} Z:{d_pos.z:6.2f}"))
            print(self.format_data("VELOCITY (m/s)", f"X:{d_vel[0]:6.2f} Y:{d_vel[1]:6.2f} Z:{d_vel[2]:6.2f}"))
            if d_lat:
                 print(self.format_data("GPS (Lat/Lon/Alt)", f"{d_lat:.6f}, {d_lon:.6f}, {d_alt:.2f}"))
            else:
                 print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for reference..."))
        else:
            print(self.format_data("LOCAL POS (m)", "Waiting..."))
            print(self.format_data("VELOCITY (m/s)", "Waiting..."))

        # --- 차량 정보 ---
        print(f"\n  {self.COLOR_BOLD}VEHICLE (X1_asp){self.COLOR_END}")
        print(self.format_data("STATE", self.vehicle_state))
        if self.vehicle_odom:
            v_pos = self.vehicle_odom.pose.pose.position
            v_vel = self.vehicle_odom.twist.twist.linear
            print(self.format_data("ODOM POS (m)", f"X:{v_pos.x:6.2f} Y:{v_pos.y:6.2f} Z:{v_pos.z:6.2f}"))
            print(self.format_data("VELOCITY (m/s)", f"X:{v_vel.x:6.2f} Y:{v_vel.y:6.2f} Z:{v_vel.z:6.2f}"))
        else:
            print(self.format_data("ODOM POS (m)", "Waiting..."))
            print(self.format_data("VELOCITY (m/s)", "Waiting..."))
        
        # 차량 GPS 변환 로직 (플레이스홀더)
        print(self.format_data("GPS (Lat/Lon/Alt)", f"{self.COLOR_YELLOW}NOT IMPLEMENTED{self.COLOR_END}"))
        
        # --- 하단 정보 ---
        timestamp = datetime.datetime.fromtimestamp(self.get_clock().now().seconds_nanoseconds()[0]).strftime('%Y-%m-%d %H:%M:%S')
        print(f"\n{self.COLOR_BOLD}{'-'*70}{self.COLOR_END}\n  Last updated: {timestamp}")


def main(args=None):
    rclpy.init(args=args)
    dashboard_node = UAVDashboard()
    try:
        rclpy.spin(dashboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
