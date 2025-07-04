#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

# 메시지 타입 임포트
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleControlMode
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray

# TF2 관련 임포트
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
    하나의 터미널에 실시간으로 표시하는 통합 대시보드 노드
    순수한 시각화 담당으로 완성된 상태 정보만 표시
    """

    def __init__(self):
        super().__init__('dashboard_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # 파라미터 선언
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

        # ANSI 색상 코드
        self.COLOR_GREEN = '\033[92m'
        self.COLOR_RED = '\033[91m'
        self.COLOR_YELLOW = '\033[93m'
        self.COLOR_CYAN = '\033[96m'
        self.COLOR_BLUE = '\033[94m'
        self.COLOR_MAGENTA = '\033[95m'
        self.COLOR_BOLD = '\033[1m'
        self.COLOR_END = '\033[0m'

        # 데이터 저장을 위한 멤버 변수
        self.drone_local_pos = None     # GPS 기준점(ref) 및 속도 정보용
        self.vehicle_odom = None        # 차량 속도 정보용
        self.vehicle_status = None      # PX4 ARM 상태
        self.vehicle_control_mode = None # PX4 제어 모드 상태
        self.detected_markers = {}      # 마커 ID별로 정보 저장: {id: {'pose': pose, 'stamp': stamp}}
        for i in range(11):  # 0 to 10
            self.detected_markers[i] = {
                'pose': None,
                'first_detection_time': None,  # 경과 시간 기록
                'is_currently_detected': False
            }
        self.drone_world_pos = None     # 드론 월드 위치 (TF)
        self.vehicle_world_pos = None   # 차량 월드 위치 (TF)
        self.drone_state = "INITIALIZING"
        self.vehicle_state = "INITIALIZING"
        
        # 미션 상태 변수 (단순히 받아서 표시만)
        self.mission_status_raw = "INIT"  # 원본 상태 문자열
        self.mission_elapsed_time = 0.0
        
        # 시스템 초기화 상태 추적
        self.system_ready = False

        # Pre-flight Check를 위한 변수
        self.topic_last_seen = {
            'PX4_LOC_POS': 0.0,
            'VEHICLE_ODOM': 0.0,
            'CAMERA_IMG': 0.0,
            'MISSION_STATUS': 0.0,
            'DRONE_STATE': 0.0,
            'VEHICLE_STATE': 0.0,
            'VEHICLE_STATUS': 0.0,
            'CONTROL_MODE': 0.0,
        }
        
        # 토픽 수신 카운터 (2번 이상 받아야 GO)
        self.topic_count = {
            'PX4_LOC_POS': 0,
            'VEHICLE_ODOM': 0,
            'CAMERA_IMG': 0,
            'VEHICLE_STATUS': 0,
            'CONTROL_MODE': 0,
        }
        self.tf_status = False
        
        # 노드 활성화 상태 추적
        self.node_status = {
            'mission_admin': False,
            'path_follower': False, 
            'offboard_control': False
        }

        # TF2 Listener 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # QoS 프로파일 정의
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

        # Subscriber 초기화 (파라미터 기반)
        self.create_subscription(VehicleLocalPosition, self.px4_topic, self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, self.vehicle_odom_topic, self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(Image, self.camera_topic, self.camera_image_callback, qos_best_effort)
        self.create_subscription(Detection3DArray, self.marker_topic, self.marker_detection_callback, qos_reliable)
        self.create_subscription(String, self.drone_state_topic, self.drone_state_callback, qos_reliable)
        self.create_subscription(String, self.vehicle_state_topic, self.vehicle_state_callback, qos_reliable)
        self.create_subscription(String, self.mission_status_topic, self.mission_status_callback, qos_reliable)
        
        # PX4 상태 모니터링을 위한 추가 구독자
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_best_effort)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_control_mode_callback, qos_best_effort)

        # 30Hz 업데이트 타이머
        self.timer = self.create_timer(1.0 / 30.0, self.update_dashboard)
        
        # 초기 화면 바로 표시
        self.draw_loading_screen()

    # 콜백 함수들
    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def drone_local_pos_callback(self, msg: VehicleLocalPosition):
        self.drone_local_pos = msg
        self.topic_last_seen['PX4_LOC_POS'] = self.get_current_time_sec()
        self.topic_count['PX4_LOC_POS'] += 1

    def vehicle_odometry_callback(self, msg: Odometry):
        self.vehicle_odom = msg
        self.topic_last_seen['VEHICLE_ODOM'] = self.get_current_time_sec()
        self.topic_count['VEHICLE_ODOM'] += 1
        
    def camera_image_callback(self, msg: Image):
        self.topic_last_seen['CAMERA_IMG'] = self.get_current_time_sec()
        self.topic_count['CAMERA_IMG'] += 1

    def marker_detection_callback(self, msg: Detection3DArray):
        # 모든 마커를 '감지되지 않음'으로 초기화
        for marker_id in self.detected_markers:
            self.detected_markers[marker_id]['is_currently_detected'] = False
            
        for det in msg.detections:
            if det.results:
                try:
                    marker_id = int(det.results[0].hypothesis.class_id)
                    if marker_id in self.detected_markers:
                        marker = self.detected_markers[marker_id]
                        marker['pose'] = det.results[0].pose.pose
                        marker['is_currently_detected'] = True
                        if marker['first_detection_time'] is None:
                            marker['first_detection_time'] = self.mission_elapsed_time
                except (ValueError, IndexError):
                    pass # 잘못된 마커 ID 형식은 무시

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

    def vehicle_status_callback(self, msg: VehicleStatus):
        """PX4 Vehicle Status 콜백 - ARM 상태 모니터링"""
        self.vehicle_status = msg
        self.topic_last_seen['VEHICLE_STATUS'] = self.get_current_time_sec()
        self.topic_count['VEHICLE_STATUS'] += 1
        
        # arming_state 값 디버깅
        if msg.arming_state != getattr(self, '_last_arming_state', -1):
            self.get_logger().info(f"Dashboard - Arming State 변경: {msg.arming_state}")
            self._last_arming_state = msg.arming_state

    def vehicle_control_mode_callback(self, msg: VehicleControlMode):
        """PX4 Vehicle Control Mode 콜백 - 제어 모드 모니터링"""
        self.vehicle_control_mode = msg
        self.topic_last_seen['CONTROL_MODE'] = self.get_current_time_sec()
        self.topic_count['CONTROL_MODE'] += 1

    # 데이터 처리 및 포맷팅
    def update_tf_poses(self):
        """TF Listener를 사용하여 드론과 차량의 월드 좌표를 업데이트"""
        try:
            # TF lookup용 완전한 프레임 ID 구성 (base_link 접미사 추가)
            full_drone_frame_id = f"{self.drone_frame_id}/base_link"
            trans_drone = self.tf_buffer.lookup_transform(self.map_frame, full_drone_frame_id, rclpy.time.Time())
            self.drone_world_pos = trans_drone.transform.translation
        except (TransformException, Exception):
            self.drone_world_pos = None

        try:
            # UGV도 동일하게 base_link 접미사 추가
            full_vehicle_frame_id = f"{self.vehicle_frame_id}/base_link"
            trans_vehicle = self.tf_buffer.lookup_transform(self.map_frame, full_vehicle_frame_id, rclpy.time.Time())
            self.vehicle_world_pos = trans_vehicle.transform.translation
        except (TransformException, Exception):
            self.vehicle_world_pos = None

    def enu_to_gps(self, x, y, z, ref_lat, ref_lon, ref_alt):
        """
        ENU 월드 좌표(map 기준)를 GPS 좌표로 변환
        PX4의 ref_lat/lon/alt를 기준점으로 사용
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

    def draw_loading_screen(self):
        """시스템 준비 전 대기화면을 ASCII 아트로 표시"""
        os.system('clear')
        
        # ASCII 아트 - KONKUK
        konkuk_art = [
            "██   ██  ██████  ███    ██ ██   ██ ██    ██ ██   ██",
            "██  ██  ██    ██ ████   ██ ██  ██  ██    ██ ██  ██ ",
            "█████   ██    ██ ██ ██  ██ █████   ██    ██ █████  ",
            "██  ██  ██    ██ ██  ██ ██ ██  ██  ██    ██ ██  ██ ",
            "██   ██  ██████  ██   ████ ██   ██  ██████  ██   ██"
        ]
        
        print(f"{self.COLOR_BOLD}{'='*70}{self.COLOR_END}")
        print()
        
        # KONKUK 출력 (중앙 정렬)
        for line in konkuk_art:
            print(f"{self.COLOR_CYAN}{line:^70}{self.COLOR_END}")
        
        print()
        print()
        
        # SMART VEHICLE ENGINEERING 출력 (일반 대문자)
        subtitle = "SMART VEHICLE ENGINEERING"
        print(f"{self.COLOR_YELLOW}{subtitle:^70}{self.COLOR_END}")
        
        print()
        print()
        
        # 로딩 상태 표시
        print(f"{self.COLOR_BOLD}{'시스템 초기화 중...':^70}{self.COLOR_END}")
        
        print()
        print(f"{self.COLOR_BOLD}{'='*70}{self.COLOR_END}")

    # 메인 업데이트 루프
    def update_dashboard(self):
        # 1. TF 정보 업데이트
        self.update_tf_poses()
        self.tf_status = bool(self.drone_world_pos)

        # 2. 노드 건강 상태 체크
        self.check_node_health()

        # 3. Pre-flight Check (2번 이상 수신되어야 GO)
        now = self.get_current_time_sec()
        px4_ok = (now - self.topic_last_seen['PX4_LOC_POS']) < self.check_timeout and self.topic_count['PX4_LOC_POS'] >= 2
        vehicle_odom_ok = (now - self.topic_last_seen['VEHICLE_ODOM']) < self.check_timeout and self.topic_count['VEHICLE_ODOM'] >= 2
        camera_ok = (now - self.topic_last_seen['CAMERA_IMG']) < self.check_timeout and self.topic_count['CAMERA_IMG'] >= 2
        
        # PX4 상태 체크
        vehicle_status_ok = (now - self.topic_last_seen['VEHICLE_STATUS']) < self.check_timeout and self.topic_count['VEHICLE_STATUS'] >= 1
        control_mode_ok = (now - self.topic_last_seen['CONTROL_MODE']) < self.check_timeout and self.topic_count['CONTROL_MODE'] >= 1
        
        # ARM 상태 체크 - VehicleControlMode의 flag_armed 사용 (더 정확함)
        is_armed = False
        if self.vehicle_control_mode:
            is_armed = self.vehicle_control_mode.flag_armed
            
        # 제어 모드 체크 (Offboard 활성화 여부)
        is_offboard = False
        if self.vehicle_control_mode:
            is_offboard = self.vehicle_control_mode.flag_control_offboard_enabled
        
        # 미션 컨트롤 시스템 전체 상태
        mission_control_ok = all(self.node_status.values())
        
        # 전체 시스템 준비 상태
        hardware_ready = all([px4_ok, vehicle_odom_ok, camera_ok, self.tf_status])
        software_ready = mission_control_ok
        px4_status_ready = all([vehicle_status_ok, control_mode_ok])
        all_systems_go = hardware_ready and software_ready and px4_status_ready
        
        # 시스템 준비 상태 확인 - 핵심 컴포넌트 중 하나라도 활성화되면 메인화면으로
        basic_ready = px4_ok or vehicle_odom_ok or self.tf_status
        if not self.system_ready and basic_ready:
            self.system_ready = True
            
        # 4. 화면 표시 분기
        if not self.system_ready:
            self.draw_loading_screen()
            return
            
        # 5. 메인 대시보드 그리기
        os.system('clear')
        
        # 섹션 1 & 2: 시스템 상태와 미션 상태를 좌우로 나란히 표시
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
        hw_status_lines = [
            f"  PX4:     [{self.COLOR_GREEN}GO{self.COLOR_END}]" if px4_ok else f"  PX4:     [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  UGV ODO: [{self.COLOR_GREEN}GO{self.COLOR_END}]" if vehicle_odom_ok else f"  UGV ODO: [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  CAM:     [{self.COLOR_GREEN}GO{self.COLOR_END}]" if camera_ok else f"  CAM:     [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  TF :     [{self.COLOR_GREEN}GO{self.COLOR_END}]" if self.tf_status else f"  TF :     [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  ARM:     [{self.COLOR_GREEN}GO{self.COLOR_END}]" if is_armed else f"  ARM:     [{self.COLOR_RED}NO-GO{self.COLOR_END}]",
            f"  OFBD_DR: [{self.COLOR_GREEN}GO{self.COLOR_END}]" if is_offboard else f"  OFBD_DR: [{self.COLOR_RED}NO-GO{self.COLOR_END}]"
        ]

        sw_status_lines = []
        node_display = {
            'mission_admin': 'ADMIN', 
            'path_follower': 'PATH',
            'offboard_control': 'OFFBD'
        }
        for node_name, status in self.node_status.items():
            display_name = node_display[node_name]
            status_str = f"[{self.COLOR_GREEN}GO{self.COLOR_END}]" if status else f"[{self.COLOR_RED}NO-GO{self.COLOR_END}]"
            sw_status_lines.append(f"{display_name}: {status_str}")

        system_lines = []
        max_sys_lines = max(len(hw_status_lines), len(sw_status_lines))
        for i in range(max_sys_lines):
            hw_part = hw_status_lines[i] if i < len(hw_status_lines) else ""
            sw_part = sw_status_lines[i] if i < len(sw_status_lines) else ""
            
            hw_clean = re.sub(r'\x1b\[[0-9;]*m', '', hw_part)
            hw_padded = hw_part + ' ' * (18 - len(hw_clean))
            
            system_lines.append(f"{hw_padded}{sw_part}")
        
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
            sys_clean = re.sub(r'\x1b\[[0-9;]*m', '', sys_line)
            mission_clean = re.sub(r'\x1b\[[0-9;]*m', '', mission_line)
            
            # 왼쪽 컬럼을 35자로 맞춤
            sys_padded = sys_line + " " * (35 - len(sys_clean))
            print(f"{sys_padded}{mission_line}")

        # 섹션 3: 마커 감지 (Marker Detections)
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'Marker Detections':^70}\n{'-'*70}{self.COLOR_END}")

        header = f"  {'ID':<4} | {'WORLD POS (m)':<25} | {'STATUS':<20} | {'FIRST SEEN':<10}"
        print(self.COLOR_BOLD + header + self.COLOR_END)

        sorted_marker_ids = sorted(self.detected_markers.keys())
        for marker_id in sorted_marker_ids:
            marker_info = self.detected_markers[marker_id]
            pos = marker_info['pose']
            first_detection_time = marker_info['first_detection_time']
            is_detected = marker_info['is_currently_detected']

            # World Position 문자열 포맷팅
            pos_str = "(-, -, -)"
            if pos:
                pos_str = f"({pos.position.x:6.2f}, {pos.position.y:6.2f}, {pos.position.z:6.2f})"
            
            # Status 및 First Seen 문자열 포맷팅
            status_str = ""
            time_str = "N/A"
            if is_detected:
                status_str = f"{self.COLOR_GREEN}DETECTED{self.COLOR_END}"
            elif first_detection_time is not None:
                status_str = f"{self.COLOR_YELLOW}UNDETECTED{self.COLOR_END}"
            else:
                status_str = "NOT SEEN"

            if first_detection_time is not None:
                time_str = f"@{first_detection_time:.1f}s"
            
            # ANSI 코드를 제외한 실제 출력 길이 계산
            status_clean_len = len(re.sub(r'\x1b\[[0-9;]*m', '', status_str))
            
            # 정렬을 위한 패딩 추가
            pos_padded = f"{pos_str:<25}"
            status_padded = f"{status_str}{' ' * (20 - status_clean_len)}"

            line = f"  {marker_id:<4} | {pos_padded} | {status_padded} | {time_str:<10}"
            print(line)
            
        # 섹션 4: 플랫폼 원격측정 (Telemetry)
        print(f"\n{self.COLOR_BOLD}{'-'*70}\n{'PLATFORM TELEMETRY':^70}\n{'-'*70}{self.COLOR_END}")
        
        # 드론 정보
        print(f"  {self.COLOR_BOLD}DRONE ({self.drone_frame_id}){self.COLOR_END}")
        
        # Line 1: State and World Pos
        state_str = f"(STATE: {self.COLOR_CYAN}{self.drone_state}{self.COLOR_END})"
        pos_str = "(WORLD: Waiting for TF...)"
        if self.drone_world_pos:
            w_pos = self.drone_world_pos
            pos_str = f"(WORLD: {self.COLOR_CYAN}{w_pos.x:6.2f}, {w_pos.y:6.2f}, {w_pos.z:6.2f}m{self.COLOR_END})"
        
        state_clean = f"(STATE: {self.drone_state})"
        state_padded = state_str + " " * (35 - len(state_clean))
        print(f"  {state_padded}{pos_str}")

        # Line 2: Velocity and GPS
        vel_str = "(VEL: Waiting for PX4...)"
        if self.drone_local_pos:
            d_vel = self.drone_local_pos
            vel_str = f"(VEL: {self.COLOR_CYAN}{d_vel.vx:5.1f}, {d_vel.vy:5.1f}, {d_vel.vz:5.1f}m/s{self.COLOR_END})"
        
        gps_str = "(GPS: N/A)"
        if self.drone_world_pos and self.drone_local_pos and self.drone_local_pos.ref_lat != 0.0:
            ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
            d_lat, d_lon, d_alt = self.enu_to_gps(self.drone_world_pos.x, self.drone_world_pos.y, self.drone_world_pos.z, ref_lat, ref_lon, ref_alt)
            if d_lat:
                gps_str = f"(GPS: {self.COLOR_CYAN}{d_lat:.6f}, {d_lon:.6f}, {d_alt:.2f}{self.COLOR_END})"
            else:
                gps_str = "(GPS: Waiting for ref...)"

        vel_clean = "(VEL: Waiting for PX4...)"
        if self.drone_local_pos:
            d_vel = self.drone_local_pos
            vel_clean = f"(VEL: {d_vel.vx:5.1f}, {d_vel.vy:5.1f}, {d_vel.vz:5.1f}m/s)"
        vel_padded = vel_str + " " * (35 - len(vel_clean))
        print(f"  {vel_padded}{gps_str}")
        
        # 차량 정보
        print(f"\n  {self.COLOR_BOLD}VEHICLE ({self.vehicle_frame_id}){self.COLOR_END}")
        
        # Line 1: State and World Pos
        v_state_str = f"(STATE: {self.COLOR_CYAN}{self.vehicle_state}{self.COLOR_END})"
        v_pos_str = "(WORLD: Waiting for TF...)"
        if self.vehicle_world_pos:
            vw_pos = self.vehicle_world_pos
            v_pos_str = f"(WORLD: {self.COLOR_CYAN}{vw_pos.x:6.2f}, {vw_pos.y:6.2f}, {vw_pos.z:6.2f}m{self.COLOR_END})"
        
        v_state_clean = f"(STATE: {self.vehicle_state})"
        v_state_padded = v_state_str + " " * (35 - len(v_state_clean))
        print(f"  {v_state_padded}{v_pos_str}")
        
        # Line 2: Velocity and GPS
        v_vel_str = "(VEL: Waiting for Odom...)"
        if self.vehicle_odom:
            v_vel = self.vehicle_odom.twist.twist.linear
            v_vel_str = f"(VEL: {self.COLOR_CYAN}{v_vel.x:5.1f}, {v_vel.y:5.1f}, {v_vel.z:5.1f}m/s{self.COLOR_END})"

        v_gps_str = "(GPS: N/A)"
        if self.vehicle_world_pos and self.drone_local_pos and self.drone_local_pos.ref_lat != 0.0:
            ref_lat, ref_lon, ref_alt = self.drone_local_pos.ref_lat, self.drone_local_pos.ref_lon, self.drone_local_pos.ref_alt
            v_lat, v_lon, v_alt = self.enu_to_gps(self.vehicle_world_pos.x, self.vehicle_world_pos.y, self.vehicle_world_pos.z, ref_lat, ref_lon, ref_alt)
            if v_lat:
                v_gps_str = f"(GPS: {self.COLOR_CYAN}{v_lat:.6f}, {v_lon:.6f}, {v_alt:.2f}{self.COLOR_END})"
            else:
                v_gps_str = "(GPS: Waiting for ref...)"
        
        v_vel_clean = "(VEL: Waiting for Odom...)"
        if self.vehicle_odom:
            v_vel = self.vehicle_odom.twist.twist.linear
            v_vel_clean = f"(VEL: {v_vel.x:5.1f}, {v_vel.y:5.1f}, {v_vel.z:5.1f}m/s)"
        
        v_vel_padded = v_vel_str + " " * (35 - len(v_vel_clean))
        print(f"  {v_vel_padded}{v_gps_str}")
        
        # --- 하단 정보 ---
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print(f"\n{self.COLOR_BOLD}{'-'*70}{self.COLOR_END}")
        print(f"  Last updated: {timestamp}")


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
