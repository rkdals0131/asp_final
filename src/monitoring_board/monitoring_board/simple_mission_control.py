#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, TakeoffStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mission_admin_interfaces.srv import MissionComplete

# --- TF2 관련 임포트 ---
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

class SimpleMissionControl(Node):
    """
    간단한 터미널 기반 미션 컨트롤.
    미션 상태를 관리하고 dashboard에 상태 정보를 발행합니다.
    ROS 시간 기반으로 동작하며 필요한 최소한의 정보만 구독합니다.
    """

    def __init__(self):
        super().__init__('simple_mission_control')
        self.set_parameters([Parameter('use_sim_time', value=True)])

        # === 파라미터 선언 ===
        self.declare_parameter('check_timeout', 2.0,
            ParameterDescriptor(description="토픽 수신 타임아웃 (초)"))
        self.declare_parameter('drone_frame_id', 'x500_gimbal_0/base_link',
            ParameterDescriptor(description="드론 TF 프레임 ID"))
        self.declare_parameter('vehicle_frame_id', 'X1_asp/base_link',
            ParameterDescriptor(description="차량 TF 프레임 ID"))
        self.declare_parameter('map_frame', 'map',
            ParameterDescriptor(description="맵 TF 프레임 ID"))

        # 파라미터 값 로드
        self.check_timeout = self.get_parameter('check_timeout').value
        self.drone_frame_id = self.get_parameter('drone_frame_id').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame = self.get_parameter('map_frame').value

        # --- 미션 상태 정의 ---
        self.MISSION_STATES = {
            'INIT': '시스템 초기화 중',
            'READY': '미션 시작 준비 완료',
            'UGV_TO_TAKEOFF': 'UGV가 이륙 지점으로 이동 중',
            'DRONE_ARMING': '드론 ARM 진행 중',
            'DRONE_TAKEOFF': '드론 이륙 중',
            'MISSION_ACTIVE': '미션 활성화 (양 플랫폼 이동)',
            'DRONE_APPROACH': '드론 랑데뷰 지점 접근',
            'DRONE_HOVER': '드론 최종 호버링',
            'MISSION_COMPLETE': '미션 완료',
            'MISSION_ABORT': '미션 중단'
        }

        # --- 미션 단계별 ID 정의 ---
        self.MISSION_IDS = {
            'UGV_TAKEOFF_ARRIVAL': 1,
            'DRONE_TAKEOFF_COMPLETE': 2,
            'UGV_MISSION_COMPLETE': 3,
            'DRONE_APPROACH_COMPLETE': 4,
            'DRONE_HOVER_COMPLETE': 5
        }

        # --- 상태 변수 ---
        self.mission_state = 'INIT'
        self.ugv_state = "INITIALIZING"
        self.drone_state = "INITIALIZING"
        self.mission_start_time = None  # ROS 시간으로 변경
        self.running = True

        # --- 플랫폼 데이터 (필요한 최소한만) ---
        self.drone_local_pos = None
        self.vehicle_odom = None

        # --- Pre-flight Check 변수 ---
        self.topic_last_seen = {
            'PX4_LOC_POS': 0.0,
            'VEHICLE_ODOM': 0.0,
        }
        self.tf_status = False

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

        # --- Subscriber 초기화 (필요한 최소한만) ---
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, "/model/X1/odometry", self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(String, "/drone/state", self.drone_state_callback, qos_reliable)
        self.create_subscription(String, "/vehicle/state", self.vehicle_state_callback, qos_reliable)

        # --- 서비스 서버 생성 ---
        self.mission_complete_srv = self.create_service(
            MissionComplete, 
            '/mission_complete', 
            self.mission_complete_callback
        )

        # --- 미션 컨트롤 퍼블리셔 ---
        self.ugv_command_pub = self.create_publisher(String, '/ugv/mission_command', 10)
        self.drone_command_pub = self.create_publisher(String, '/drone/mission_command', 10)
        
        # --- 미션 상태 퍼블리셔 (Dashboard용) ---
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)

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

    def signal_handler(self, signum, frame):
        """Ctrl+C 핸들러"""
        self.running = False
        self.get_logger().info("종료 중...")
        
    # --- 콜백 함수들 ---
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
            self.get_logger().info(f"🚁 드론 상태: {self.drone_state} -> {msg.data}")
        self.drone_state = msg.data

    def vehicle_state_callback(self, msg: String):
        if self.ugv_state != msg.data:
            self.get_logger().info(f"🚗 UGV 상태: {self.ugv_state} -> {msg.data}")
        self.ugv_state = msg.data

    def mission_complete_callback(self, request, response):
        """미션 완료 신호 처리 - 단순화된 상태 머신"""
        mission_id = request.mission_id
        self.get_logger().info(f"🔧 Debug: 미션 완료 신호 수신 - ID: {mission_id}, 현재 상태: {self.mission_state}")
        
        # 상태 전이 로직을 단순한 매핑으로 정리
        state_transitions = {
            self.MISSION_IDS['UGV_TAKEOFF_ARRIVAL']: {
                'expected_states': ['UGV_TO_TAKEOFF'],
                'next_state': 'DRONE_ARMING',
                'action': lambda: self.drone_command_pub.publish(String(data='start')),
                'message': "✅ UGV가 이륙 위치에 도착. 드론 시작 명령 전송"
            },
            self.MISSION_IDS['DRONE_TAKEOFF_COMPLETE']: {
                'expected_states': ['DRONE_ARMING', 'DRONE_TAKEOFF'],
                'next_state': 'MISSION_ACTIVE',
                'action': lambda: [
                    self.ugv_command_pub.publish(String(data='resume')),
                    self.drone_command_pub.publish(String(data='start'))
                ],
                'message': "✅ 드론 이륙 완료. UGV resume 시작"
            },
            self.MISSION_IDS['UGV_MISSION_COMPLETE']: {
                'expected_states': ['MISSION_ACTIVE', 'DRONE_APPROACH', 'DRONE_HOVER', 'MISSION_COMPLETE'],
                'next_state': None,  # 상태 변경 없음
                'action': None,
                'message': "✅ UGV 미션 완료"
            },
            self.MISSION_IDS['DRONE_APPROACH_COMPLETE']: {
                'expected_states': ['MISSION_ACTIVE', 'DRONE_APPROACH', 'DRONE_HOVER', 'MISSION_COMPLETE'],
                'next_state': 'DRONE_HOVER',
                'action': None,
                'message': "✅ 드론 랑데뷰 지점 도착"
            },
            self.MISSION_IDS['DRONE_HOVER_COMPLETE']: {
                'expected_states': ['DRONE_HOVER', 'MISSION_COMPLETE'],
                'next_state': 'MISSION_COMPLETE',
                'action': None,
                'message': "🎯 미션 완료!"
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
                self.get_logger().warn(f"❌ 미션 ID {mission_id} 거부: 현재 상태 {self.mission_state} not in {transition['expected_states']}")
                response.success = False
        else:
            self.get_logger().warn(f"❌ 알 수 없는 미션 ID: {mission_id}")
            response.success = False
            
        self.get_logger().info(f"🔧 Debug: 응답 - success: {response.success}")
        return response

    def publish_mission_status(self):
        """미션 상태를 Dashboard에 발행 - ROS 시간 기반"""
        status_msg = String()
        if self.mission_start_time:
            elapsed_ns = (self.get_clock().now() - self.mission_start_time).nanoseconds
            elapsed_sec = elapsed_ns / 1e9
            status_msg.data = f"{self.mission_state}|{elapsed_sec:.1f}"
        else:
            status_msg.data = f"{self.mission_state}|0.0"
        self.mission_status_pub.publish(status_msg)

    def update_tf_poses(self):
        """TF Listener를 사용하여 시스템 상태 확인"""
        try:
            trans_drone = self.tf_buffer.lookup_transform(self.map_frame, self.drone_frame_id, rclpy.time.Time())
            self.tf_status = True
        except TransformException:
            self.tf_status = False

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
                self.get_logger().info("🟢 시스템 준비 완료! 's' 키를 눌러 미션을 시작하세요.")
        
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
            print(f"🎯 미션 상태: {self.mission_state} | 경과시간: {elapsed_sec:.1f}초")
        else:
            print(f"🎯 미션 상태: {self.mission_state}")

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
                        print(f"❌ 현재 상태({self.mission_state})에서는 미션을 시작할 수 없습니다.")
                elif command == 'a':
                    self.abort_mission()
                elif command == 'r':
                    self.reset_mission()
                else:
                    print("❌ 알 수 없는 명령어입니다. 's', 'a', 'r', 'q' 중 하나를 입력하세요.")
                    
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
        self.get_logger().info("🚀 미션 시작! UGV가 이륙 지점으로 이동합니다.")

    def abort_mission(self):
        """미션 중단"""
        self.mission_state = 'MISSION_ABORT'
        
        self.ugv_command_pub.publish(String(data='stop'))
        self.drone_command_pub.publish(String(data='land'))
        self.publish_mission_status()
        self.get_logger().warn("⛔ 미션 중단!")

    def reset_mission(self):
        """미션 상태 리셋"""
        self.mission_state = 'READY'
        self.mission_start_time = None
        self.publish_mission_status()
        self.get_logger().info("🔄 미션 상태 리셋")


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