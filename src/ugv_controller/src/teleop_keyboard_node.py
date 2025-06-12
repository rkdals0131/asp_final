#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading
import time

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # Publisher 생성
        self.publisher = self.create_publisher(Twist, '/model/X1_asp/cmd_vel', 10)
        
        # 속도 설정
        self.linear_speed = 1.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        self.key_timeout = 0.2 # 키 입력이 없을 때 멈춤으로 간주할 시간 (초)

        # Twist 메시지 초기화
        self.twist = Twist()
        
        # 터미널 원본 설정 저장
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 동시 키 입력을 처리하기 위한 상태 저장 변수
        # 각 키의 마지막 입력 시간을 저장합니다.
        self.last_key_press_time = {}
        
        # 제어 안내 출력
        self.print_instructions()
        
        # 키보드 입력 및 Twist 메시지 발행을 위한 스레드/타이머
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # 20Hz (0.05초마다) 주기로 키 상태를 확인하고 Twist 메시지를 발행
        self.update_timer = self.create_timer(0.05, self.update_and_publish_twist)

    def print_instructions(self):
        """제어 방법과 현재 속도를 터미널에 출력합니다."""
        print("\n=== UGV Teleop Keyboard Control (개선판) ===")
        print("      w: 전진")
        print("a: 좌회전 | s: 후진 | d: 우회전")
        print("w+a, w+d 등으로 대각선 이동 가능")
        print("---------------------------------")
        print("↑/↓: 선형속도 조절 | ←/→: 각속도 조절")
        print("---------------------------------")
        print("Space: 비상 정지 | q 또는 Ctrl+C: 종료")
        print("=====================================")
        self.print_current_speeds()

    def print_current_speeds(self):
        """현재 설정된 최대 속도를 한 줄에 출력합니다."""
        # \r: 커서를 줄의 시작으로 이동시켜, 다음 출력으로 덮어쓰게 함
        print(f"\r[최대 속도] 선형: {self.linear_speed:.1f} m/s | 각속도: {self.angular_speed:.1f} rad/s   ", end="")
        sys.stdout.flush()

    def keyboard_input_loop(self):
        """
        키보드 입력을 감지하여 마지막 입력 시간을 기록합니다.
        """
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == '\x1b': # 방향키 입력 처리 (2바이트 추가 읽기)
                        key += sys.stdin.read(2)

                    if key == 'q' or key == '\x03': # 종료
                        self.running = False
                        break
                    
                    # 속도 조절 키 (즉시 반영)
                    elif key == '\x1b[A': # 위
                        self.linear_speed = min(self.linear_speed + self.speed_increment, 2.0)
                        self.print_current_speeds()
                    elif key == '\x1b[B': # 아래
                        self.linear_speed = max(self.linear_speed - self.speed_increment, 0.1)
                        self.print_current_speeds()
                    elif key == '\x1b[C': # 오른쪽
                        self.angular_speed = min(self.angular_speed + self.speed_increment, 2.0)
                        self.print_current_speeds()
                    elif key == '\x1b[D': # 왼쪽
                        self.angular_speed = max(self.angular_speed - self.speed_increment, 0.1)
                        self.print_current_speeds()
                    
                    # 움직임 키 (마지막 입력 시간 기록)
                    elif key in ['w', 's', 'a', 'd']:
                        self.last_key_press_time[key] = time.time()
                    
                    # 비상 정지 키
                    elif key == ' ':
                        self.last_key_press_time.clear() # 모든 키 상태 초기화

        except Exception as e:
            self.get_logger().error(f"키보드 입력 처리 중 오류 발생: {e}")
        finally:
            # 스레드 종료 시 터미널 설정을 반드시 원래대로 복원
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def update_and_publish_twist(self):
        """
        키 입력 상태를 바탕으로 Twist 메시지를 계산하고 발행합니다.
        키 입력이 self.key_timeout 동안 없으면 해당 키는 떨어진 것으로 간주합니다.
        """
        if not self.running:
            return

        now = time.time()
        linear_x = 0.0
        angular_z = 0.0

        # 키가 마지막으로 눌린 시간이 timeout 이내인지 확인하여 현재 눌림 상태를 판단
        w_pressed = (now - self.last_key_press_time.get('w', 0)) < self.key_timeout
        s_pressed = (now - self.last_key_press_time.get('s', 0)) < self.key_timeout
        a_pressed = (now - self.last_key_press_time.get('a', 0)) < self.key_timeout
        d_pressed = (now - self.last_key_press_time.get('d', 0)) < self.key_timeout

        # 상태에 따라 속도 계산 (동시 입력 처리)
        if w_pressed:
            linear_x = self.linear_speed
        elif s_pressed:
            linear_x = -self.linear_speed

        if a_pressed:
            angular_z = self.angular_speed
        elif d_pressed:
            angular_z = -self.angular_speed

        # 계산된 속도로 Twist 메시지 업데이트 및 발행
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.publisher.publish(self.twist)

    def destroy_node(self):
        """노드 종료 시 모든 것을 안전하게 정리합니다."""
        print("\nTeleop 노드를 종료합니다. 터미널 설정을 복원합니다.")
        self.running = False
        # 스레드가 완전히 종료될 때까지 잠시 대기
        self.keyboard_thread.join(timeout=0.2)
        # 마지막으로 정지 명령을 보냄
        self.publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboardNode()
    
    try:
        rclpy.spin(teleop_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
