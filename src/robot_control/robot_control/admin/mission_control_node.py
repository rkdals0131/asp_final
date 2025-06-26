#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

# 메시지 타입 임포트
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, TakeoffStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from mission_admin_interfaces.srv import MissionComplete
from visualization_msgs.msg import MarkerArray

# TF2 관련 임포트
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import os
import datetime
import math
import threading
import signal
import sys
from typing import Optional, Dict, List

# 리팩터링된 viz_factory 임포트
from robot_control.utils import viz_factory

class SimpleMissionControl(Node):
    """
    간단한 터미널 기반 미션 컨트롤
    미션 상태를 관리하고 dashboard에 상태 정보를 발행
    ROS 시간 기반으로 동작하며 필요한 최소한의 정보만 구독
    """

    def __init__(self):
        super().__init__('mission_control_node')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # 파라미터 선언
        self.declare_parameter('check_timeout', 2.0,
            ParameterDescriptor(description="토픽 수신 타임아웃 (초)"))
        self.declare_parameter('drone_frame_id', 'x500_gimbal_0',
            ParameterDescriptor(description="드론 TF 프레임 ID"))
        self.declare_parameter('vehicle_frame_id', 'X1_asp',
            ParameterDescriptor(description="차량 TF 프레임 ID"))
        self.declare_parameter('map_frame', 'map',
            ParameterDescriptor(description="맵 TF 프레임 ID"))
        # Ground Truth CSV 파일 경로 파라미터 추가
        self.declare_parameter('ground_truth_csv_path', 'config/aruco_markers.csv',
            ParameterDescriptor(description="Ground Truth 마커 CSV 파일 경로"))

        # 파라미터 값 로드
        self.check_timeout = self.get_parameter('check_timeout').value
        self.drone_frame_id = self.get_parameter('drone_frame_id').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame = self.get_parameter('map_frame').value

        # 미션 상태 정의
        self.MISSION_STATES = {
            'INIT': '시스템 초기화 중',
            'READY': '미션 시작 준비 완료',
            'UGV_TO_TAKEOFF': 'UGV가 이륙 지점으로 이동 중',
            'DRONE_ARMING': '드론 ARM 진행 중',
            'DRONE_TAKEOFF': '드론 이륙 중',
            'MISSION_ACTIVE': '미션 활성화 (양 플랫폼 이동)',
            'LANDING_STANDBY': '드론/UGV 착륙지점 대기',
            'PRECISION_LANDING': '정밀 착륙 진행 중',
            'MISSION_COMPLETE': '미션 완료',
            'MISSION_ABORT': '미션 중단'
        }

        # 미션 단계별 ID 정의
        self.MISSION_IDS = {
            'UGV_ARM_POSITION_ARRIVAL': 1,  # mission_type 1 도달 시 드론 ARM
            'UGV_TAKEOFF_ARRIVAL': 2,       # mission_type 2 도달 시 드론 takeoff
            'UGV_MISSION_COMPLETE': 3,      # UGV 미션 완료
            'DRONE_TAKEOFF_COMPLETE': 4,    # 드론 이륙 완료
            'DRONE_APPROACH_COMPLETE': 5,   # 드론 접근 완료
            'DRONE_HOVER_COMPLETE': 6,      # 드론 호버링 완료
            'DRONE_WP8_ARRIVAL': 7          # 드론 WP8 도달
        }

        # 상태 변수
        self.mission_state = 'INIT'
        self.ugv_state = "INITIALIZING"
        self.drone_state = "INITIALIZING"
        self.mission_start_time = None  # ROS 시간으로 변경
        self.running = True

        # 미션 플래그
        self.ugv_ready_for_landing = False
        self.drone_ready_for_landing = False
        self.landing_command_sent = False
        self.freefall_command_sent = False
        self.mission_end_time = None

        # 플랫폼 데이터 (필요한 최소한만)
        self.drone_local_pos = None
        self.vehicle_odom = None

        # Pre-flight Check 변수
        self.topic_last_seen = {
            'PX4_LOC_POS': 0.0,
            'VEHICLE_ODOM': 0.0,
        }
        self.tf_status = False

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

        # Subscriber 초기화 (필요한 최소한만)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, "/model/X1/odometry", self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(String, "/drone/state", self.drone_state_callback, qos_reliable)
        self.create_subscription(String, "/vehicle/state", self.vehicle_state_callback, qos_reliable)

        # 서비스 서버 생성
        self.mission_complete_srv = self.create_service(
            MissionComplete, 
            '/mission_complete', 
            self.mission_complete_callback
        )

        # 미션 컨트롤 퍼블리셔
        self.ugv_command_pub = self.create_publisher(String, '/ugv/mission_command', 10)
        self.drone_command_pub = self.create_publisher(String, '/drone/mission_command', 10)
        self.marker_detector_command_pub = self.create_publisher(String, '/multi_tracker/command', 10)
        
        # 미션 상태 퍼블리셔 (Dashboard용)
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)

        # Ground-Truth 마커 발행 로직 추가
        self._publish_ground_truth_markers()

        # 데이터 업데이트 타이머
        self.timer = self.create_timer(1.0 / 10.0, self.update_data)  # 10Hz
        self.status_timer = self.create_timer(5.0, self.print_status)  # 5초마다 간단한 상태 출력
        
        # 키보드 입력 스레드
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info("=== Simple Mission Control Dashboard v4.0 ===")
        self.get_logger().info("ROS 시간 기반, 최적화된 통신 구조")
        self.get_logger().info("명령어: 's'=시작, 'a'=중단, 'r'=리셋, 'q'=종료")

    def _publish_ground_truth_markers(self):
        """
        노드 초기화 시 viz_factory를 호출하여 Ground Truth 마커를
        Latched 토픽으로 한 번만 발행.
        """
        gt_csv_path = self.get_parameter('ground_truth_csv_path').get_parameter_value().string_value
        
        # 패키지 루트를 기준으로 절대 경로 생성
        if not os.path.isabs(gt_csv_path):
            package_dir = os.path.join(os.path.dirname(__file__), '..', '..')
            gt_csv_path = os.path.join(package_dir, gt_csv_path)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        
        # viz_factory의 고수준 함수 호출
        marker_array, info_msg = viz_factory.create_ground_truth_markers_from_csv(header, gt_csv_path)

        if not marker_array.markers:
            self.get_logger().error(info_msg) # 실패 시 info_msg는 에러 메시지
            return

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gt_marker_pub = self.create_publisher(MarkerArray, '/ground_truth_markers', latched_qos)
        
        gt_marker_pub.publish(marker_array)
        self.get_logger().info(info_msg) # 성공 시 info_msg는 로드 정보

    def signal_handler(self, signum, frame):
        """Ctrl+C 핸들러"""
        self.running = False
        self.get_logger().info("종료 중...")
        
    # 콜백 함수들
    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def drone_local_pos_callback(self, msg: VehicleLocalPosition):
        self.drone_local_pos = msg
        self.topic_last_seen['PX4_LOC_POS'] = self.get_current_time_sec()

    def vehicle_odometry_callback(self, msg: Odometry):
        self.vehicle_odom = msg
        self.topic_last_seen['VEHICLE_ODOM'] = self.get_current_time_sec()

    def drone_state_callback(self, msg: String):
        if self.drone_state != msg.data:
            self.get_logger().info(f"드론 상태: {self.drone_state} -> {msg.data}")
        self.drone_state = msg.data

        # 드론이 Disarmed 상태가 되면 미션 완료 처리
        if self.mission_state in ['PRECISION_LANDING', 'MISSION_COMPLETE'] and self.drone_state == 'DISARMED':
            if self.mission_state != 'MISSION_COMPLETE':
                self.mission_state = 'MISSION_COMPLETE'
                self.mission_end_time = self.get_clock().now()
                self.publish_mission_status()
                self.get_logger().info("미션 완료! 드론 Disarmed 확인")

    def vehicle_state_callback(self, msg: String):
        if self.ugv_state != msg.data:
            self.get_logger().info(f"UGV 상태: {self.ugv_state} -> {msg.data}")
            
            # UGV가 COMPLETE 상태가 되면 랑데부 준비 완료 플래그 설정
            if msg.data == "COMPLETE" and not self.ugv_ready_for_landing:
                self.get_logger().info("UGV 랑데부 준비 완료")
                self.ugv_ready_for_landing = True
                self._check_and_start_landing()
                
        self.ugv_state = msg.data

    def mission_complete_callback(self, request, response):
        """미션 완료 신호 처리 - 단순화된 상태 머신"""
        mission_id = request.mission_id
        self.get_logger().info(f"Debug: 미션 완료 신호 수신 - ID: {mission_id}, 현재 상태: {self.mission_state}")
        
        # 상태 전이 로직을 단순한 매핑으로 정리
        state_transitions = {
            self.MISSION_IDS['UGV_ARM_POSITION_ARRIVAL']: {
                'expected_states': ['UGV_TO_TAKEOFF'],
                'next_state': 'DRONE_ARMING',
                'action': lambda: self.drone_command_pub.publish(String(data='start')),
                'message': "UGV가 ARM 위치에 도착. 드론 ARM 명령 전송"
            },
            self.MISSION_IDS['UGV_TAKEOFF_ARRIVAL']: {
                'expected_states': ['DRONE_ARMING', 'DRONE_TAKEOFF'],
                'next_state': 'DRONE_TAKEOFF',
                'action': lambda: self.drone_command_pub.publish(String(data='takeoff')),
                'message': "UGV가 이륙 위치에 도착. 드론 즉시 takeoff 명령 전송"
            },
            self.MISSION_IDS['DRONE_TAKEOFF_COMPLETE']: {
                'expected_states': ['DRONE_ARMING', 'DRONE_TAKEOFF'],
                'next_state': 'MISSION_ACTIVE',
                'action': lambda: [
                    self.ugv_command_pub.publish(String(data='resume')),
                    self.drone_command_pub.publish(String(data='start'))
                ],
                'message': "드론 이륙 완료. UGV resume 시작"
            },
            self.MISSION_IDS['DRONE_WP8_ARRIVAL']: {
                'expected_states': ['MISSION_ACTIVE'],
                'next_state': None,  # 상태 변경 없음, 미션은 계속 활성 상태
                'action': self._send_freefall_command,
                'message': "드론 WP8 도착. 자유낙하 명령 전송"
            },
            self.MISSION_IDS['UGV_MISSION_COMPLETE']: {
                'expected_states': ['MISSION_ACTIVE', 'DRONE_APPROACH', 'DRONE_HOVER', 'MISSION_COMPLETE'],
                'next_state': None,  # 상태 변경 없음
                'action': None,
                'message': "UGV 미션 완료"
            },
            self.MISSION_IDS['DRONE_APPROACH_COMPLETE']: {
                'expected_states': ['MISSION_ACTIVE', 'LANDING_STANDBY'],
                'next_state': 'LANDING_STANDBY',
                'action': self._set_drone_ready_for_landing,
                'message': "드론 최종 지점 도착. UGV 도착 대기"
            },
            self.MISSION_IDS['DRONE_HOVER_COMPLETE']: {
                'expected_states': ['LANDING_STANDBY', 'PRECISION_LANDING', 'MISSION_COMPLETE'],
                'next_state': 'MISSION_COMPLETE',
                'action': None,
                'message': "미션 완료!"
            }
        }
        
        if mission_id in state_transitions:
            transition = state_transitions[mission_id]
            
            # 현재 상태가 예상 상태와 일치하는지 확인
            if self.mission_state in transition['expected_states']:
                # 상태 변경
                if transition['next_state'] and transition['next_state'] != self.mission_state:
                    self.mission_state = transition['next_state']
                    self.publish_mission_status()
                
                # 액션 실행
                if transition['action']:
                    if isinstance(transition['action'](), list):
                        # 여러 액션의 경우
                        pass  # 이미 실행됨
                    else:
                        transition['action']()
                
                self.get_logger().info(transition['message'])
                response.success = True
            else:
                self.get_logger().warn(f"미션 ID {mission_id} 거부: 현재 상태 {self.mission_state} not in {transition['expected_states']}")
                response.success = False
        else:
            self.get_logger().warn(f"알 수 없는 미션 ID: {mission_id}")
            response.success = False
            
        self.get_logger().info(f"Debug: 응답 - success: {response.success}")
        return response

    def publish_mission_status(self):
        """미션 상태를 Dashboard에 발행 - ROS 시간 기반"""
        status_msg = String()
        elapsed_sec = 0.0

        if self.mission_state == 'MISSION_COMPLETE' and self.mission_end_time:
            # 미션 완료 시, 시작부터 종료까지의 시간으로 경과시간 고정
            if self.mission_start_time:
                elapsed_ns = (self.mission_end_time - self.mission_start_time).nanoseconds
                elapsed_sec = elapsed_ns / 1e9
        elif self.mission_start_time:
            # 미션 진행 중, 현재 시간 기준으로 경과시간 계산
            elapsed_ns = (self.get_clock().now() - self.mission_start_time).nanoseconds
            elapsed_sec = elapsed_ns / 1e9

        status_msg.data = f"{self.mission_state}|{elapsed_sec:.1f}"
        self.mission_status_pub.publish(status_msg)

    def update_tf_poses(self):
        """TF 정보 업데이트 (간단한 상태 체크용)"""
        try:
            # TF lookup용 완전한 프레임 ID 구성 (base_link 접미사 추가)
            full_drone_frame_id = f"{self.drone_frame_id}/base_link"
            trans_drone = self.tf_buffer.lookup_transform(self.map_frame, full_drone_frame_id, rclpy.time.Time())
            # 필요시 추가 정보 저장
        except TransformException:
            pass

        try:
            # UGV도 동일하게 base_link 접미사 추가
            full_vehicle_frame_id = f"{self.vehicle_frame_id}/base_link"
            trans_vehicle = self.tf_buffer.lookup_transform(self.map_frame, full_vehicle_frame_id, rclpy.time.Time())
            # 필요시 추가 정보 저장
        except TransformException:
            pass

    def update_data(self):
        """데이터 업데이트 및 미션 상태 확인 - ROS 시간 기반"""
        self.update_tf_poses()
        
        # 시스템 준비 상태 확인
        if self.mission_state == 'INIT':
            now = self.get_current_time_sec()
            px4_ok = (now - self.topic_last_seen['PX4_LOC_POS']) < self.check_timeout
            vehicle_odom_ok = (now - self.topic_last_seen['VEHICLE_ODOM']) < self.check_timeout
            
            if all([px4_ok, vehicle_odom_ok]):
                self.mission_state = 'READY'
                self.get_logger().info("시스템 준비 완료! 's' 키를 눌러 미션을 시작하세요")
        
        elif self.mission_state == 'DRONE_ARMING':
            if self.drone_state in ['ARMED_IDLE', 'TAKING_OFF']:
                self.mission_state = 'DRONE_TAKEOFF'
                self.publish_mission_status()

        # 주기적으로 미션 상태 발행
        self.publish_mission_status()

    def print_status(self):
        """간단한 상태 출력 (로깅 최소화) - ROS 시간 기반"""
        if self.mission_start_time:
            elapsed_ns = (self.get_clock().now() - self.mission_start_time).nanoseconds
            elapsed_sec = elapsed_ns / 1e9
            print(f"미션 상태: {self.mission_state} | 경과시간: {elapsed_sec:.1f}초")
        else:
            print(f"미션 상태: {self.mission_state}")

    def input_loop(self):
        """키보드 입력 처리 루프"""
        print("\n명령어: 's'=시작, 'a'=중단, 'r'=리셋, 'q'=종료")
        
        while self.running:
            try:
                command = input().strip().lower()
                
                if command == 'q':
                    self.running = False
                    break
                elif command == 's':
                    if self.mission_state == 'READY':
                        self.start_mission()
                    else:
                        print(f"현재 상태({self.mission_state})에서는 미션을 시작할 수 없습니다")
                elif command == 'a':
                    self.abort_mission()
                elif command == 'r':
                    self.reset_mission()
                else:
                    print("알 수 없는 명령어입니다. 's', 'a', 'r', 'q' 중 하나를 입력하세요")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                break

    def start_mission(self):
        """미션 시작 - ROS 시간 기반"""
        self.mission_state = 'UGV_TO_TAKEOFF'
        self.mission_start_time = self.get_clock().now()  # ROS 시간으로 기록
        
        self.ugv_command_pub.publish(String(data='go'))
        self.publish_mission_status()
        self.get_logger().info("미션 시작! UGV가 이륙 지점으로 이동")

    def abort_mission(self):
        """미션 중단"""
        self.mission_state = 'MISSION_ABORT'
        
        self.ugv_command_pub.publish(String(data='stop'))
        self.drone_command_pub.publish(String(data='land'))
        self.publish_mission_status()
        self.get_logger().warn("미션 중단!")

    def reset_mission(self):
        """미션 상태 리셋"""
        self.mission_state = 'READY'
        self.mission_start_time = None
        self.mission_end_time = None
        self.ugv_ready_for_landing = False
        self.drone_ready_for_landing = False
        self.landing_command_sent = False
        self.freefall_command_sent = False
        self.publish_mission_status()
        self.get_logger().info("미션 상태 리셋")

    def _set_drone_ready_for_landing(self):
        """드론 랑데부 준비완료 플래그를 설정하고, 착륙 시작 조건을 확인"""
        if not self.drone_ready_for_landing:
            self.get_logger().info("드론 랑데부 준비 완료")
            self.drone_ready_for_landing = True
            self._check_and_start_landing()

    def _check_and_start_landing(self):
        """UGV와 드론이 모두 준비되었는지 확인하고 정밀 착륙을 시작"""
        if self.ugv_ready_for_landing and self.drone_ready_for_landing and not self.landing_command_sent:
            self.get_logger().info("모든 플랫폼 준비 완료. 정밀 착륙 시퀀스 시작!")
            self.mission_state = 'PRECISION_LANDING'
            self.publish_mission_status()
            
            # 드론에 정밀 착륙 시작 명령 전송
            self.drone_command_pub.publish(String(data='start_precision_landing'))
            
            # 마커 탐지기에 탐지 시작 명령 전송
            self.marker_detector_command_pub.publish(String(data='DETECT_LANDING_MARKER'))
            
            self.landing_command_sent = True

    def _send_freefall_command(self):
        """자유낙하 명령을 한 번만 보내도록 제어"""
        if not self.freefall_command_sent:
            self.drone_command_pub.publish(String(data='start_freefall'))
            self.freefall_command_sent = True
        else:
            self.get_logger().info("자유낙하 명령은 이미 전송되었습니다. 중복 전송 방지.")


def main(args=None):
    rclpy.init(args=args)
    control_node = SimpleMissionControl()
    
    try:
        while control_node.running and rclpy.ok():
            rclpy.spin_once(control_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.running = False
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 