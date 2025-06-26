#!/usr/bin/env python3
"""
드론 동역학 캘리브레이션 노드 (단순화 버전)
자동 추력 특성 테스트 및 고주파수 데이터 수집
"""
import rclpy
import threading
import sys
import copy
import time
import os
from datetime import datetime

from px4_msgs.msg import VehicleLandDetected, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped

from .base_mission_node import BaseMissionNode
from robot_control.utils import drone_control_utils as dcu


class InteractiveCalibrationNode(BaseMissionNode):
    """
    드론 동역학 캘리브레이션을 위한 단순화된 노드
    
    기능:
    - 자동 추력 특성 테스트 (0.0~1.0, 각 5초)
    - 100Hz 고주파수 데이터 수집
    - 자동 로그 파일 생성
    """
    
    def __init__(self):
        super().__init__('interactive_calibration_node', drone_frame_id="x500_gimbal_0")
        
        # 추가 서브스크라이버
        self.land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self.land_detected_callback, self.qos_profile
        )
        
        # 캘리브레이션 전용 상태 변수
        self.land_detected = None
        self.takeoff_altitude = 5.0
        self.takeoff_target_local = None
        
        # 테스트 파라미터
        self.test_params = {}
        self.data_logger = dcu.ManeuverDataLogger()
        self.logging_active = False
        

        
        # 사용자 입력 스레드
        self.input_thread = threading.Thread(target=self.command_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        self.get_logger().info("🔬 드론 추력 특성 캘리브레이션 노드 시작")
        self.get_logger().info("💡 명령어: start → takeoff → test → land")
    
    # 콜백 함수
    
    def land_detected_callback(self, msg: VehicleLandDetected):
        """착륙 감지 콜백"""
        self.land_detected = msg
    
    # 사용자 입력 처리
    
    def command_input_loop(self):
        """간단한 사용자 명령 입력"""
        print("\n🔬 드론 캘리브레이션 콘솔 (단순화)")
        print("=" * 50)
        print("  start                - 드론 시동 및 ARM")
        print("  takeoff              - 이륙 (100m)")
        print("  maintain <altitude>  - 지정 고도로 이동/유지")
        print("  test <thrust> [duration] - 개별 추력 테스트 (0.0~1.0, 기본 5초)")
        print("    예: test 0.2, test 0.5 10.0")
        print("  log start            - 데이터 로깅 시작")
        print("  log stop             - 데이터 로깅 중지")
        print("  log save <filename>  - CSV 파일로 저장")
        print("  land                 - 착륙")
        print("  stop                 - 현재 작업 중지")
        print("=" * 50)
        
        single_commands = {
            "start": self._handle_start_command,
            "takeoff": self._handle_takeoff_command,
            "land": self._handle_land_command,
            "stop": self._handle_stop_command,
        }
        
        for line in sys.stdin:
            cmd = line.strip().split()
            if not cmd:
                continue
            command = cmd[0].lower()
            
            # 단일 명령어 처리
            if command in single_commands:
                single_commands[command]()
            
            # 인자가 필요한 명령어들
            elif command == "maintain" and len(cmd) > 1:
                self._handle_maintain_command(cmd[1])
            elif command == "test" and len(cmd) > 1:
                self._handle_test_command(cmd[1:])
            elif command == "log" and len(cmd) > 1:
                self._handle_log_command(cmd[1:])
            else:
                self.get_logger().warn(f"알 수 없는 명령: '{line.strip()}'")
    
    def _handle_start_command(self):
        """시작 명령 처리"""
        if self.state == "INIT":
            self.get_logger().info("🚀 캘리브레이션 시작! 드론 시동 및 ARM")
            self.start_mission()
        elif self.state == "LANDED":
            self.get_logger().info("🔄 재시동: ARM 및 Offboard 모드 전환")
            dcu.arm_and_offboard(self)
            self.state = "ARMED_IDLE"
        else:
            self.get_logger().warn(f"START 명령을 사용할 수 없는 상태: {self.state}")
    
    def _handle_takeoff_command(self):
        """이륙 명령 처리"""
        if self.state == "ARMED_IDLE":
            self.get_logger().info("🛫 이륙 시작")
            self.state = "TAKING_OFF"
        else:
            self.get_logger().warn(f"이륙할 수 없는 상태: {self.state}")
    
    def _handle_maintain_command(self, altitude_str):
        """고도 유지 명령 처리"""
        try:
            if self.state not in ["IDLE", "TESTING"]:
                self.get_logger().warn(f"고도 유지를 할 수 없는 상태: {self.state}")
                return
            
            target_altitude = float(altitude_str)
            
            if target_altitude < 5.0:
                self.get_logger().warn("안전을 위해 최소 5m 이상의 고도를 유지하세요")
                return
            
            if target_altitude > 2000.0:
                self.get_logger().warn("안전을 위해 최대 2000m 이하의 고도를 유지하세요")
                return
            
            if self.current_map_pose:
                current_pos = self.current_map_pose.pose.position
                
                # 현재 테스트 중지 후 이동
                self.test_params = {}
                self.target_altitude = target_altitude
                
                self.get_logger().info(f"🎯 목표 고도 {target_altitude:.1f}m로 이동/유지")
                self.state = "MOVING_TO_ALTITUDE"
            else:
                self.get_logger().warn("현재 위치를 알 수 없어 고도 변경 불가")
                
        except ValueError:
            self.get_logger().error(f"잘못된 고도 값: {altitude_str}")
    
    def _handle_test_command(self, args):
        """개별 추력 테스트 시작"""
        try:
            if self.state != "IDLE":
                self.get_logger().warn(f"테스트를 시작할 수 없는 상태: {self.state}")
                return
            
            thrust_value = float(args[0])
            duration = 5.0  # 기본 지속 시간 (초)
            if len(args) > 1:
                duration = float(args[1])
            
            if thrust_value < 0.0 or thrust_value > 1.0:
                self.get_logger().error("추력값은 0.0~1.0 사이여야 합니다")
                return
            
            self.get_logger().info(f"🧪 추력 {thrust_value:.2f} 테스트 시작 ({duration:.1f}초)")
            
            self.test_params = {
                'type': 'thrust',
                'thrust_value': thrust_value,
                'start_time': time.time(),
                'duration': duration
            }
            self.state = "TESTING"
            
        except (ValueError, IndexError):
            self.get_logger().error(f"잘못된 추력 또는 시간 값: {args}")
    
    def _handle_log_command(self, args):
        """로깅 명령 처리"""
        if not args:
            return
        
        subcommand = args[0].lower()
        
        if subcommand == "start":
            if not self.logging_active:
                self.data_logger = dcu.ManeuverDataLogger()
                self.data_logger.start_logging()
                self.logging_active = True
                self.get_logger().info("📊 데이터 로깅 시작 (100Hz)")
            else:
                self.get_logger().warn("이미 로깅이 진행 중입니다")
        
        elif subcommand == "stop":
            if self.logging_active:
                self.logging_active = False
                self.get_logger().info("📊 데이터 로깅 중지")
            else:
                self.get_logger().warn("로깅이 진행 중이 아닙니다")
        
        elif subcommand == "save" and len(args) > 1:
            filename = args[1]
            if not filename.endswith('.csv'):
                filename += '.csv'
            
            # 결과 디렉토리 생성
            results_dir = os.path.join(os.getcwd(), 'calibration_results')
            os.makedirs(results_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            full_filename = f"{timestamp}_{filename}"
            filepath = os.path.join(results_dir, full_filename)
            
            if self.data_logger.save_to_csv(filepath):
                self.get_logger().info(f"💾 데이터가 저장되었습니다: {filepath}")
                stats = self.data_logger.get_summary_stats()
                if stats:
                    self.get_logger().info(f"📈 총 데이터 포인트: {stats.get('data_points', 0)}개")
            else:
                self.get_logger().error("저장할 데이터가 없습니다")
    
    def _handle_land_command(self):
        """착륙 명령 처리"""
        if self.state in ["IDLE", "TESTING", "MOVING_TO_ALTITUDE"]:
            self.get_logger().info("🛬 착륙 시작")
            self.test_params = {}
            self.state = "LANDING"
        else:
            self.get_logger().warn(f"착륙할 수 없는 상태: {self.state}")
    
    def _handle_stop_command(self):
        """정지 명령 처리"""
        if self.state in ["IDLE", "TESTING", "MOVING_TO_ALTITUDE"]:
            self.get_logger().info("⏹️  모든 테스트 정지")
            self.test_params = {}
            self.state = "IDLE"
        else:
            self.get_logger().warn(f"정지할 수 없는 상태: {self.state}")
    
    # 미션 로직 구현
    
    def run_mission_logic(self):
        """캘리브레이션 노드의 상태 머신 로직"""
        
        # 고주파수 데이터 로깅 (100Hz) - run_mission_logic이 약 100Hz로 호출됨
        if self.logging_active and self.current_local_pos and self.current_map_pose:
            timestamp = time.time()
            altitude = self.current_map_pose.pose.position.z
            velocity = getattr(self.current_local_pos, 'vz', 0.0)
            acceleration = getattr(self.current_local_pos, 'az', 0.0)
            
            # 현재 모터 출력
            motor_outputs = self.test_params.get('current_motor_outputs', [0.5, 0.5, 0.5, 0.5])
            control_info = self.test_params.get('control_info', {})
            
            self.data_logger.log_data_point(timestamp, altitude, velocity, acceleration, motor_outputs, control_info)
        
        # 상태별 처리
        if self.state == "HANDSHAKE":
            self.get_logger().info("🤝 핸드셰이크 진행 중...", throttle_duration_sec=2.0)
        elif self.state == "ARMED_IDLE":
            self.get_logger().info("🔧 ARM 완료. 'takeoff' 명령으로 이륙하세요.", throttle_duration_sec=3.0)
        elif self.state == "TAKING_OFF":
            self._handle_takeoff_state()
        elif self.state == "MOVING_TO_ALTITUDE":
            self._handle_moving_to_altitude_state()
        elif self.state == "TESTING":
            self._handle_testing_state()
        elif self.state == "IDLE":
            self._handle_idle_state()
        elif self.state == "LANDING":
            self._handle_landing_state()
    
    def _handle_takeoff_state(self):
        """이륙 상태 처리"""
        if self.takeoff_target_local is None:
            self.get_logger().info(f"🚁 이륙 시작. 목표 고도: {self.takeoff_altitude}m")
            self.takeoff_target_local = [
                self.current_local_pos.x,
                self.current_local_pos.y,
                self.current_local_pos.z - self.takeoff_altitude
            ]
        
        # 이륙 세트포인트 퍼블리시
        sp_msg = TrajectorySetpoint(
            position=[float(p) for p in self.takeoff_target_local],
            timestamp=int(self.get_clock().now().nanoseconds / 1000)
        )
        self.trajectory_setpoint_publisher.publish(sp_msg)
        
        # 이륙 완료 확인
        if abs(self.current_local_pos.z - self.takeoff_target_local[2]) < 2.0:
            self.get_logger().info("✅ 이륙 완료. 테스트 준비됨")
            self.state = "IDLE"
            self.takeoff_target_local = None
    
    def _handle_moving_to_altitude_state(self):
        """고도 이동 상태 처리"""
        if hasattr(self, 'target_altitude') and self.current_map_pose:
            current_pos = self.current_map_pose.pose.position
            target_pos = [current_pos.x, current_pos.y, self.target_altitude]
            
            # 위치 제어로 고도 이동
            dcu.publish_position_setpoint(self, target_pos)
            
            # 도착 확인 (1m 허용 오차)
            altitude_error = abs(current_pos.z - self.target_altitude)
            if altitude_error < 1.0:
                self.get_logger().info(f"✅ 목표 고도 도착: {current_pos.z:.1f}m")
                self.state = "IDLE"
    
    def _handle_testing_state(self):
        """테스트 상태 처리 - 저수준 제어"""
        test_type = self.test_params.get('type')
        current_time = time.time()
        
        if test_type == "thrust":
            self._execute_thrust_test(current_time)
    
    def _execute_thrust_test(self, current_time):
        """추력 테스트 실행 - Interactive mission 방식"""
        # 1. 액추에이터 직접 제어 모드 활성화
        dcu.publish_offboard_control_mode(self, 
                                          position=False, 
                                          velocity=False, 
                                          acceleration=False,
                                          attitude=False, 
                                          body_rate=False,
                                          thrust_and_torque=False,
                                          actuator=True)

        thrust_value = self.test_params['thrust_value']
        elapsed = current_time - self.test_params['start_time']
        max_duration = self.test_params.get('duration', 5.0)
        
        # 시간 완료 확인
        if elapsed > max_duration:
            self.get_logger().info(f"✅ 추력 {thrust_value:.1f} 테스트 완료")
            self.test_params = {}
            self.state = "IDLE"
            return
        
        # 고정 추력 적용
        motor_outputs = [thrust_value] * 4
        
        # 로깅용 데이터 저장
        self.test_params['current_motor_outputs'] = motor_outputs
        self.test_params['control_info'] = {
            'test_type': 'thrust', 
            'thrust_value': thrust_value,
            'elapsed': elapsed
        }
        
        # 모터 출력 직접 발행
        dcu.publish_actuator_motors(self, motor_outputs)
        
        # 진행 상황 출력 (1초마다)
        if int(current_time * 2) % 4 == 0:
            current_altitude = self.current_map_pose.pose.position.z
            self.get_logger().info(f"⚡ 추력: {thrust_value:.1f} | 고도: {current_altitude:.1f}m | 경과: {elapsed:.1f}s")
    
    def _handle_idle_state(self):
        """대기 상태 처리 - 위치 제어로 안정화"""
        if self.current_map_pose:
            current_pos = self.current_map_pose.pose.position
            target_pos = [current_pos.x, current_pos.y, current_pos.z]
            dcu.publish_position_setpoint(self, target_pos)
    
    def _handle_landing_state(self):
        """착륙 상태 처리"""
        self.get_logger().info("🛬 착륙 명령 실행 중...", throttle_duration_sec=2.0)
        dcu.land_drone(self)
        
        if self.land_detected and self.land_detected.landed:
            self.get_logger().info("✅ 착륙 완료. 'start' 명령으로 재시동 가능")
            self.state = "LANDED"
            self.test_params = {}
            self.logging_active = False


def main(args=None):
    rclpy.init(args=args)
    calibration_node = InteractiveCalibrationNode()
    
    try:
        rclpy.spin(calibration_node)
        
    except (KeyboardInterrupt, SystemExit):
        calibration_node.get_logger().info("🛑 종료 요청. 강제 착륙")
        if hasattr(calibration_node, 'state') and calibration_node.state not in ["LANDED", "INIT"]:
            dcu.land_drone(calibration_node)
    finally:
        if rclpy.ok():
            calibration_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main() 