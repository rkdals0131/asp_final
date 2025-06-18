#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

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
import re

class UAVDashboard(Node):
    """
    드론과 차량의 시스템 상태, 임무 정보, 원격측정 데이터를
    하나의 터미널에 실시간으로 표시하는 통합 대시보드 노드.
    순수한 시각화 담당으로 완성된 상태 정보만 표시합니다.
    """

    def __init__(self):
        super().__init__('dashboard_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # === 파라미터 선언 ===
        self.declare_parameter('px4_local_pos_topic', '/fmu/out/vehicle_local_position',
            ParameterDescriptor(description="PX4 로컬 위치 토픽"))
        self.declare_parameter('vehicle_odom_topic', '/model/X1/odometry',
            ParameterDescriptor(description="차량 오도메트리 토픽"))
        self.declare_parameter('camera_image_topic', '/x500/image_proc',
            ParameterDescriptor(description="카메라 이미지 토픽"))
        self.declare_parameter('marker_detection_topic', '/marker_detections',
            ParameterDescriptor(description="마커 감지 토픽"))
        self.declare_parameter('drone_state_topic', '/drone/state',
            ParameterDescriptor(description="드론 상태 토픽"))
        self.declare_parameter('vehicle_state_topic', '/vehicle/state',
            ParameterDescriptor(description="차량 상태 토픽"))
        self.declare_parameter('mission_status_topic', '/mission/status',
            ParameterDescriptor(description="미션 상태 토픽"))
        
        self.declare_parameter('drone_frame_id', 'x500_gimbal_0',
            ParameterDescriptor(description="드론 TF 프레임 ID"))
        self.declare_parameter('vehicle_frame_id', 'X1_asp',
            ParameterDescriptor(description="차량 TF 프레임 ID"))
        self.declare_parameter('map_frame', 'map',
            ParameterDescriptor(description="맵 TF 프레임 ID"))
        
        self.declare_parameter('check_timeout', 2.0,
            ParameterDescriptor(description="토픽 수신 타임아웃 (초)"))
        self.declare_parameter('node_timeout', 5.0,
            ParameterDescriptor(description="노드 활성화 체크 타임아웃 (초)"))

        # 파라미터 값 로드
        self.px4_topic = self.get_parameter('px4_local_pos_topic').value
        self.vehicle_odom_topic = self.get_parameter('vehicle_odom_topic').value
        self.camera_topic = self.get_parameter('camera_image_topic').value
        self.marker_topic = self.get_parameter('marker_detection_topic').value
        self.drone_state_topic = self.get_parameter('drone_state_topic').value
        self.vehicle_state_topic = self.get_parameter('vehicle_state_topic').value
        self.mission_status_topic = self.get_parameter('mission_status_topic').value
        
        self.drone_frame_id = self.get_parameter('drone_frame_id').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame = self.get_parameter('map_frame').value
        
        self.check_timeout = self.get_parameter('check_timeout').value
        self.node_timeout = self.get_parameter('node_timeout').value

        # --- ANSI 색상 코드 ---
        self.COLOR_GREEN = '\033[92m'
        self.COLOR_RED = '\033[91m'
        self.COLOR_YELLOW = '\033[93m'
        self.COLOR_CYAN = '\033[96m'
        self.COLOR_BLUE = '\033[94m'
        self.COLOR_MAGENTA = '\033[95m'
        self.COLOR_BOLD = '\033[1m'
        self.COLOR_END = '\033[0m'

        # --- 데이터 저장을 위한 멤버 변수 ---
        self.drone_local_pos = None     # GPS 기준점(ref) 및 속도 정보용
        self.vehicle_odom = None        # 차량 속도 정보용
        self.marker_detections = None   # 마커 정보
        self.drone_world_pos = None     # 드론 월드 위치 (TF)
        self.vehicle_world_pos = None   # 차량 월드 위치 (TF)
        self.drone_state = "INITIALIZING"
        self.vehicle_state = "INITIALIZING"
        
        # --- 미션 상태 변수 (단순히 받아서 표시만) ---
        self.mission_status_raw = "INIT"  # 원본 상태 문자열
        self.mission_elapsed_time = 0.0

        # --- Pre-flight Check를 위한 변수 ---
        self.topic_last_seen = {
            'PX4_LOC_POS': 0.0,
            'VEHICLE_ODOM': 0.0,
            'CAMERA_IMG': 0.0,
            'MISSION_STATUS': 0.0,
            'DRONE_STATE': 0.0,
            'VEHICLE_STATE': 0.0,
        }
        self.tf_status = False
        
        # --- 노드 활성화 상태 추적 ---
        self.node_status = {
            'mission_admin': False,
            'path_follower': False, 
            'offboard_control': False
        }

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
            depth=10
        )

        # --- Subscriber 초기화 (파라미터 기반) ---
        self.create_subscription(VehicleLocalPosition, self.px4_topic, self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, self.vehicle_odom_topic, self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(Image, self.camera_topic, self.camera_image_callback, qos_best_effort)
        self.create_subscription(Detection3DArray, self.marker_topic, self.marker_detection_callback, qos_reliable)
        self.create_subscription(String, self.drone_state_topic, self.drone_state_callback, qos_reliable)
        self.create_subscription(String, self.vehicle_state_topic, self.vehicle_state_callback, qos_reliable)
        self.create_subscription(String, self.mission_status_topic, self.mission_status_callback, qos_reliable)

        # 30Hz 업데이트 타이머
        self.timer = self.create_timer(1.0 / 30.0, self.update_dashboard)
        
        self.get_logger().info("UAV Telemetry Dashboard v3.0 초기화 완료 - 순수 시각화 모드")

    # --- 콜백 함수들 ---
    def get_current_time_sec(self):
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
        self.topic_last_seen['DRONE_STATE'] = self.get_current_time_sec()
        self.node_status['offboard_control'] = True

    def vehicle_state_callback(self, msg: String):
        self.vehicle_state = msg.data
        self.topic_last_seen['VEHICLE_STATE'] = self.get_current_time_sec()
        self.node_status['path_follower'] = True

    def mission_status_callback(self, msg: String):
        """미션 상태 콜백 - 완성된 정보만 받아서 표시"""
        self.topic_last_seen['MISSION_STATUS'] = self.get_current_time_sec()
        self.node_status['mission_admin'] = True
        
        try:
            # 메시지 형식: "STATE|elapsed_time" or "STATE"
            parts = msg.data.split('|')
            if len(parts) >= 2:
                self.mission_status_raw = parts[0]
                self.mission_elapsed_time = float(parts[1])
            else:
                self.mission_status_raw = msg.data
                self.mission_elapsed_time = 0.0
        except:
            self.mission_status_raw = msg.data
            self.mission_elapsed_time = 0.0

    # --- 데이터 처리 및 포맷팅 ---
    def update_tf_poses(self):
        """TF Listener를 사용하여 드론과 차량의 월드 좌표를 업데이트합니다."""
        try:
            # TF lookup용 완전한 프레임 ID 구성 (base_link 접미사 추가)
            full_drone_frame_id = f"{self.drone_frame_id}/base_link"
            trans_drone = self.tf_buffer.lookup_transform(self.map_frame, full_drone_frame_id, rclpy.time.Time())
            self.drone_world_pos = trans_drone.transform.translation
        except TransformException:
            self.drone_world_pos = None

        try:
            # UGV도 동일하게 base_link 접미사 추가
            full_vehicle_frame_id = f"{self.vehicle_frame_id}/base_link"
            trans_vehicle = self.tf_buffer.lookup_transform(self.map_frame, full_vehicle_frame_id, rclpy.time.Time())
            self.vehicle_world_pos = trans_vehicle.transform.translation
        except TransformException:
            self.vehicle_world_pos = None

    def enu_to_gps(self, x, y, z, ref_lat, ref_lon, ref_alt):
        """
        ENU 월드 좌표(map 기준)를 GPS 좌표로 변환합니다.
        PX4의 ref_lat/lon/alt를 기준점으로 사용합니다.
        """
        if ref_lat == 0.0 or ref_lon == 0.0:
            return None, None, None
            
        earth_radius = 6378137.0
        
        # ENU to GPS
        # X: East, Y: North, Z: Up
        d_lon = math.degrees(x / (earth_radius * math.cos(math.radians(ref_lat))))
        d_lat = math.degrees(y / earth_radius)
        
        new_lat = ref_lat + d_lat
        new_lon = ref_lon + d_lon
        new_alt = ref_alt + z
        
        return new_lat, new_lon, new_alt

    def format_status(self, label, is_ok):
        status_str = f"{self.COLOR_GREEN}GO  {self.COLOR_END}" if is_ok else f"{self.COLOR_RED}NO-GO{self.COLOR_END}"
        return f"  {label:<18}: [{status_str}]"
    
    def format_data(self, label, value, unit=""):
        return f"  {label:<18}: {self.COLOR_CYAN}{value}{self.COLOR_END} {unit}"

    def get_mission_status_color(self):
        """미션 상태에 따른 색상 반환 (간단한 키워드 기반)"""
        status_lower = self.mission_status_raw.lower()
        if 'complete' in status_lower:
            return self.COLOR_GREEN
        elif 'abort' in status_lower or 'error' in status_lower:
            return self.COLOR_RED
        elif 'ready' in status_lower:
            return self.COLOR_YELLOW
        elif 'active' in status_lower or 'moving' in status_lower or 'takeoff' in status_lower:
            return self.COLOR_BLUE
        else:
            return self.COLOR_CYAN

    def check_node_health(self):
        """노드 건강 상태를 체크하여 스마트한 GO/NO-GO 판단"""
        now = self.get_current_time_sec()
        
        # 각 노드의 활성화 상태 체크
        for node_name in self.node_status:
            if node_name == 'mission_admin':
                self.node_status[node_name] = (now - self.topic_last_seen['MISSION_STATUS']) < self.node_timeout
            elif node_name == 'path_follower':
                self.node_status[node_name] = (now - self.topic_last_seen['VEHICLE_STATE']) < self.node_timeout
            elif node_name == 'offboard_control':
                self.node_status[node_name] = (now - self.topic_last_seen['DRONE_STATE']) < self.node_timeout

    # --- 메인 업데이트 루프 ---
    def update_dashboard(self):
        # 1. TF 정보 업데이트
        self.update_tf_poses()
        self.tf_status = bool(self.drone_world_pos)

        # 2. 노드 건강 상태 체크
        self.check_node_health()

        # 3. Pre-flight Check
        now = self.get_current_time_sec()
        px4_ok = (now - self.topic_last_seen['PX4_LOC_POS']) < self.check_timeout
        vehicle_odom_ok = (now - self.topic_last_seen['VEHICLE_ODOM']) < self.check_timeout
        camera_ok = (now - self.topic_last_seen['CAMERA_IMG']) < self.check_timeout
        
        # 미션 컨트롤 시스템 전체 상태
        mission_control_ok = all(self.node_status.values())
        
        # 전체 시스템 준비 상태
        hardware_ready = all([px4_ok, vehicle_odom_ok, camera_ok, self.tf_status])
        software_ready = mission_control_ok
        all_systems_go = hardware_ready and software_ready

        # 4. 화면 클리어 및 대시보드 그리기
        os.system('clear')
        
        # === 섹션 1 & 2: 시스템 상태와 미션 상태를 좌우로 나란히 표시 ===
        print(f"{self.COLOR_BOLD}{'='*70}{self.COLOR_END}")
        
        # 헤더 라인
        status_header = "SYSTEMS STATUS"
        mission_header = "MISSION STATUS"
        print(f"{self.COLOR_BOLD}{status_header:^35}{mission_header:^35}{self.COLOR_END}")
        print(f"{self.COLOR_BOLD}{'-'*35}{'-'*35}{self.COLOR_END}")
        
        # 상태 요약 라인
        hw_status = f"HW:{self.COLOR_GREEN}GO{self.COLOR_END}" if hardware_ready else f"HW:{self.COLOR_RED}NO-GO{self.COLOR_END}"
        sw_status = f"SW:{self.COLOR_GREEN}GO{self.COLOR_END}" if software_ready else f"SW:{self.COLOR_RED}NO-GO{self.COLOR_END}"
        all_go_str = f"({hw_status}|{sw_status})"
        
        mission_color = self.get_mission_status_color()
        mission_summary = f"{mission_color}{self.mission_status_raw}{self.COLOR_END}"
        
        print(f"{all_go_str:^40}{mission_summary:^40}")
        print()
        
        # 시스템 상태 세부 정보 (왼쪽)
        system_lines = [
            f"  PX4: [{self.COLOR_GREEN}GO{self.COLOR_END}]" if px4_ok else f"  PX4: [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  UGV ODO: [{self.COLOR_GREEN}GO{self.COLOR_END}]" if vehicle_odom_ok else f"  UGV ODO: [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  CAM: [{self.COLOR_GREEN}GO{self.COLOR_END}]" if camera_ok else f"  CAM: [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  TF : [{self.COLOR_GREEN}GO{self.COLOR_END}]" if self.tf_status else f"  TF : [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            ""  # 빈 줄
        ]
        
        # 노드 상태 추가
        for node_name, status in self.node_status.items():
            node_display = {
                'mission_admin': 'ADMIN', 
                'path_follower': 'PATH',
                'offboard_control': 'OFFBD'
            }
            display_name = node_display[node_name]
            status_str = f"[{self.COLOR_GREEN}GO{self.COLOR_END}]" if status else f"[{self.COLOR_RED}NO-GO{self.COLOR_END}]"
            system_lines.append(f"  {display_name}: {status_str}")
        
        # 미션 정보 (오른쪽) - 단순히 받은 정보만 표시
        mission_lines = []
        
        # 총 경과시간
        if self.mission_elapsed_time > 0:
            mission_lines.append(f"  경과시간: {self.COLOR_CYAN}{self.mission_elapsed_time:.1f}초{self.COLOR_END}")
        else:
            mission_lines.append(f"  경과시간: {self.COLOR_YELLOW}대기중{self.COLOR_END}")
        
        # 플랫폼 개별 상태
        mission_lines.append(f"  UGV: {self.COLOR_CYAN}{self.vehicle_state}{self.COLOR_END}")
        mission_lines.append(f"  UAV: {self.COLOR_CYAN}{self.drone_state}{self.COLOR_END}")
        mission_lines.append("")  # 빈 줄
        
        # 미션 상태 상세 정보
        mission_lines.append(f"  현재 단계:")
        mission_lines.append(f"    {mission_color}{self.mission_status_raw}{self.COLOR_END}")
        
        # 빈 줄로 패딩하여 같은 높이로 맞춤
        max_lines = max(len(system_lines), len(mission_lines))
        while len(system_lines) < max_lines:
            system_lines.append(" " * 15)
        while len(mission_lines) < max_lines:
            mission_lines.append(" " * 15)
        
        # 좌우로 나란히 출력
        for sys_line, mission_line in zip(system_lines, mission_lines):
            # ANSI 코드를 제외한 실제 텍스트 길이 계산
            sys_clean = re.sub(r'\033\[[0-9;]*m', '', sys_line)
            mission_clean = re.sub(r'\033\[[0-9;]*m', '', mission_line)
            
            # 왼쪽 컬럼을 35자로 맞춤
            sys_padded = sys_line + " " * (35 - len(sys_clean))
            print(f"{sys_padded}{mission_line}")

        # === 섹션 3: 임무 정보 (Marker Detections) ==========================
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'MISSION PAYLOAD (MARKER DETECTIONS)':^70}\n{'-'*70}{self.COLOR_END}")
        if self.marker_detections and self.drone_local_pos:
            if not self.marker_detections.detections:
                print(f"  {self.COLOR_YELLOW}Searching for markers...{self.COLOR_END}")
            else:
                ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
                
                for det in self.marker_detections.detections:
                    marker_id = det.results[0].hypothesis.class_id
                    pos = det.results[0].pose.pose.position
                    m_lat, m_lon, m_alt = self.enu_to_gps(pos.x, pos.y, pos.z, ref_lat, ref_lon, ref_alt)
                    
                    print(f"  {self.COLOR_BOLD}MARKER ID: {marker_id}{self.COLOR_END}")
                    print(f"    - WORLD POS (m) : {self.COLOR_CYAN}X:{pos.x:6.2f} Y:{pos.y:6.2f} Z:{pos.z:6.2f}{self.COLOR_END}")
                    if m_lat:
                        print(f"    - GPS EST (Lat/Lon/Alt) : {self.COLOR_CYAN}{m_lat:.6f}, {m_lon:.6f}, {m_alt:.2f}{self.COLOR_END}")
                    else:
                        print(f"    - GPS EST         : {self.COLOR_YELLOW}Waiting for drone's GPS reference...{self.COLOR_END}")
        else:
            print(f"  {self.COLOR_YELLOW}Waiting for detection or GPS ref data...{self.COLOR_END}")
            
        # === 섹션 4: 플랫폼 원격측정 (Telemetry) =========================
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'PLATFORM TELEMETRY':^70}\n{'-'*70}{self.COLOR_END}")
        
        # --- 드론 정보 ---
        print(f"  {self.COLOR_BOLD}DRONE ({self.drone_frame_id}){self.COLOR_END}")
        print(self.format_data("STATE", self.drone_state))
        if self.drone_world_pos:
            w_pos = self.drone_world_pos
            print(self.format_data("WORLD POS (m)", f"X:{w_pos.x:6.2f} Y:{w_pos.y:6.2f} Z:{w_pos.z:6.2f}"))
        else:
            print(self.format_data("WORLD POS (m)", "Waiting for TF..."))

        if self.drone_local_pos:
            d_vel = self.drone_local_pos.vx, self.drone_local_pos.vy, self.drone_local_pos.vz
            print(self.format_data("VELOCITY (m/s)", f"X:{d_vel[0]:6.2f} Y:{d_vel[1]:6.2f} Z:{d_vel[2]:6.2f}"))

            if self.drone_world_pos:
                ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
                d_lat, d_lon, d_alt = self.enu_to_gps(self.drone_world_pos.x, self.drone_world_pos.y, self.drone_world_pos.z, ref_lat, ref_lon, ref_alt)
                if d_lat:
                    print(self.format_data("GPS (Lat/Lon/Alt)", f"{d_lat:.6f}, {d_lon:.6f}, {d_alt:.2f}"))
                else:
                    print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for GPS reference..."))
            else:
                print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for World Pos..."))
        else:
            print(self.format_data("VELOCITY (m/s)", "Waiting for PX4..."))
            print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for PX4..."))

        # --- 차량 정보 ---
        print(f"\n  {self.COLOR_BOLD}VEHICLE ({self.vehicle_frame_id}){self.COLOR_END}")
        print(self.format_data("STATE", self.vehicle_state))
        if self.vehicle_world_pos:
            vw_pos = self.vehicle_world_pos
            print(self.format_data("WORLD POS (m)", f"X:{vw_pos.x:6.2f} Y:{vw_pos.y:6.2f} Z:{vw_pos.z:6.2f}"))
        else:
            print(self.format_data("WORLD POS (m)", "Waiting for TF..."))
            
        if self.vehicle_odom:
            v_vel = self.vehicle_odom.twist.twist.linear
            print(self.format_data("VELOCITY (m/s)", f"X:{v_vel.x:6.2f} Y:{v_vel.y:6.2f} Z:{v_vel.z:6.2f}"))
        else:
            print(self.format_data("VELOCITY (m/s)", "Waiting for Odom..."))
        
        if self.vehicle_world_pos and self.drone_local_pos:
            ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
            v_lat, v_lon, v_alt = self.enu_to_gps(self.vehicle_world_pos.x, self.vehicle_world_pos.y, self.vehicle_world_pos.z, ref_lat, ref_lon, ref_alt)
            if v_lat:
                print(self.format_data("GPS (Lat/Lon/Alt)", f"{v_lat:.6f}, {v_lon:.6f}, {v_alt:.2f}"))
            else:
                print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for GPS reference..."))
        else:
            print(self.format_data("GPS (Lat/Lon/Alt)", "Waiting for World Pos or GPS ref..."))
        
        # --- 하단 정보 ---
        timestamp = datetime.datetime.fromtimestamp(self.get_clock().now().seconds_nanoseconds()[0]).strftime('%Y-%m-%d %H:%M:%S')
        print(f"\n{self.COLOR_BOLD}{'-'*70}{self.COLOR_END}")
        print(f"  Last updated: {timestamp}")
        if mission_control_ok:
            print(f"  {self.COLOR_GREEN}✅ Mission Control Connected{self.COLOR_END}")
        else:
            print(f"  {self.COLOR_RED}❌ Mission Control Disconnected{self.COLOR_END}")


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
