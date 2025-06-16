#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- 메시지 타입 임포트 ---
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, TakeoffStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray
from mission_admin_interfaces.srv import MissionComplete

# --- TF2 관련 임포트 ---
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import os
import datetime
import math
import threading
import time
import signal
import sys
from typing import Optional, Dict, List

class SimpleMissionControl(Node):
    """
    간단한 터미널 기반 미션 컨트롤.
    미션 상태를 관리하고 dashboard에 상태 정보를 발행합니다.
    """

    def __init__(self):
        super().__init__('simple_mission_control')
        self.set_parameters([Parameter('use_sim_time', value=True)])

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
        self.mission_start_time = None
        self.last_status_time = time.time()
        self.running = True

        # --- 플랫폼 데이터 ---
        self.drone_local_pos = None
        self.vehicle_odom = None
        self.marker_detections = None
        self.drone_world_pos = None
        self.vehicle_world_pos = None

        # --- Pre-flight Check 변수 ---
        self.check_timeout = 2.0
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
            depth=10
        )

        # --- Subscriber 초기화 ---
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.drone_local_pos_callback, qos_best_effort)
        self.create_subscription(Odometry, "/model/X1/odometry", self.vehicle_odometry_callback, qos_reliable)
        self.create_subscription(Image, "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image", self.camera_image_callback, qos_best_effort)
        self.create_subscription(Detection3DArray, "/marker_detections", self.marker_detection_callback, qos_reliable)
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

        self.get_logger().info("=== Simple Mission Control Dashboard v3.2 ===")
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
        
    def camera_image_callback(self, msg: Image):
        self.topic_last_seen['CAMERA_IMG'] = self.get_current_time_sec()

    def marker_detection_callback(self, msg: Detection3DArray):
        self.marker_detections = msg

    def drone_state_callback(self, msg: String):
        if self.drone_state != msg.data:
            self.get_logger().info(f"🚁 드론 상태: {self.drone_state} -> {msg.data}")
        self.drone_state = msg.data

    def vehicle_state_callback(self, msg: String):
        if self.ugv_state != msg.data:
            self.get_logger().info(f"🚗 UGV 상태: {self.ugv_state} -> {msg.data}")
        self.ugv_state = msg.data

    def mission_complete_callback(self, request, response):
        """미션 완료 신호 처리"""
        mission_id = request.mission_id
        self.get_logger().info(f"🔧 Debug: 미션 완료 신호 수신 - ID: {mission_id}, 현재 상태: {self.mission_state}")
        
        if mission_id == self.MISSION_IDS['UGV_TAKEOFF_ARRIVAL']:
            if self.mission_state == 'UGV_TO_TAKEOFF':
                self.mission_state = 'DRONE_ARMING'
                self.get_logger().info("✅ UGV가 이륙 위치에 도착. 드론 시작 명령 전송")
                self.drone_command_pub.publish(String(data='start'))
                self.publish_mission_status()
                response.success = True
            else:
                self.get_logger().warn(f"❌ UGV_TAKEOFF_ARRIVAL 거부: 현재 상태 {self.mission_state} != UGV_TO_TAKEOFF")
                response.success = False
                
        elif mission_id == self.MISSION_IDS['DRONE_TAKEOFF_COMPLETE']:
            if self.mission_state in ['DRONE_ARMING', 'DRONE_TAKEOFF']:
                self.mission_state = 'MISSION_ACTIVE'
                self.get_logger().info("✅ 드론 이륙 완료. UGV resume 시작")
                self.ugv_command_pub.publish(String(data='resume'))
                self.drone_command_pub.publish(String(data='start'))
                self.publish_mission_status()
                response.success = True
            else:
                self.get_logger().warn(f"❌ DRONE_TAKEOFF_COMPLETE 거부: 현재 상태 {self.mission_state} not in [DRONE_ARMING, DRONE_TAKEOFF]")
                response.success = False
                
        elif mission_id == self.MISSION_IDS['UGV_MISSION_COMPLETE']:
            self.get_logger().info("✅ UGV 미션 완료")
            response.success = True
                
        elif mission_id == self.MISSION_IDS['DRONE_APPROACH_COMPLETE']:
            if self.mission_state in ['MISSION_ACTIVE', 'DRONE_APPROACH', 'DRONE_HOVER', 'MISSION_COMPLETE']:
                # 이미 완료된 상태여도 OK - 중복 신호일 수 있음
                if self.mission_state not in ['DRONE_HOVER', 'MISSION_COMPLETE']:
                    self.mission_state = 'DRONE_HOVER'
                    self.publish_mission_status()
                self.get_logger().info("✅ 드론 랑데뷰 지점 도착")
                response.success = True
            else:
                self.get_logger().warn(f"❌ DRONE_APPROACH_COMPLETE 거부: 현재 상태 {self.mission_state} - 예상하지 못한 상태")
                response.success = False
                
        elif mission_id == self.MISSION_IDS['DRONE_HOVER_COMPLETE']:
            if self.mission_state in ['DRONE_HOVER', 'MISSION_COMPLETE']:
                # 이미 완료된 상태여도 OK - 중복 신호일 수 있음
                if self.mission_state != 'MISSION_COMPLETE':
                    self.mission_state = 'MISSION_COMPLETE'
                    self.get_logger().info("🎯 미션 완료!")
                    self.publish_mission_status()
                else:
                    self.get_logger().info("🎯 미션 완료 (이미 완료됨)")
                response.success = True
            else:
                self.get_logger().warn(f"❌ DRONE_HOVER_COMPLETE 거부: 현재 상태 {self.mission_state} - 예상하지 못한 상태")
                response.success = False
        else:
            self.get_logger().warn(f"❌ 알 수 없는 미션 ID: {mission_id}")
            response.success = False
            
        self.get_logger().info(f"🔧 Debug: 응답 - success: {response.success}")
        return response

    def publish_mission_status(self):
        """미션 상태를 Dashboard에 발행"""
        status_msg = String()
        if self.mission_start_time:
            elapsed = time.time() - self.mission_start_time
            status_msg.data = f"{self.mission_state}|{elapsed:.1f}"
        else:
            status_msg.data = f"{self.mission_state}|0.0"
        self.mission_status_pub.publish(status_msg)

    def update_tf_poses(self):
        """TF Listener를 사용하여 드론과 차량의 월드 좌표를 업데이트합니다."""
        try:
            trans_drone = self.tf_buffer.lookup_transform(self.map_frame, self.drone_frame_id, rclpy.time.Time())
            self.drone_world_pos = trans_drone.transform.translation
        except TransformException:
            self.drone_world_pos = None

        try:
            trans_vehicle = self.tf_buffer.lookup_transform(self.map_frame, self.vehicle_frame_id, rclpy.time.Time())
            self.vehicle_world_pos = trans_vehicle.transform.translation
        except TransformException:
            self.vehicle_world_pos = None

    def update_data(self):
        """데이터 업데이트 및 미션 상태 확인"""
        self.update_tf_poses()
        self.tf_status = bool(self.drone_world_pos)
        
        # 시스템 준비 상태 확인
        if self.mission_state == 'INIT':
            now = self.get_current_time_sec()
            px4_ok = (now - self.topic_last_seen['PX4_LOC_POS']) < self.check_timeout
            vehicle_odom_ok = (now - self.topic_last_seen['VEHICLE_ODOM']) < self.check_timeout
            camera_ok = (now - self.topic_last_seen['CAMERA_IMG']) < self.check_timeout
            
            # TF 체크는 선택사항으로 변경 (시뮬레이션 환경에 따라)
            if all([px4_ok, vehicle_odom_ok, camera_ok]):
                self.mission_state = 'READY'
                self.get_logger().info("🟢 시스템 준비 완료! 's' 키를 눌러 미션을 시작하세요.")
        
        elif self.mission_state == 'DRONE_ARMING':
            if self.drone_state in ['ARMED_IDLE', 'TAKING_OFF']:
                self.mission_state = 'DRONE_TAKEOFF'
                self.publish_mission_status()

        # 주기적으로 미션 상태 발행
        self.publish_mission_status()

    def print_status(self):
        """간단한 상태 출력 (로깅 최소화)"""
        if self.mission_start_time:
            elapsed = time.time() - self.mission_start_time
            print(f"🎯 미션 상태: {self.mission_state} | 경과시간: {elapsed:.1f}초")
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
        """미션 시작"""
        self.mission_state = 'UGV_TO_TAKEOFF'
        self.mission_start_time = time.time()
        
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