# **[기능 개발 PRD] PID 제어를 이용한 고속 수직 강하 기능**

*   **문서 버전:** 1.0
*   **작성일:** 2025년 6월 26일
*   **담당:** Gemini (AI Assistant)
*   **상태:** 제안 (Proposal)

---

## 1. 개요 (Overview)

### 1.1. 문제 정의 (Problem Statement)
현재 드론은 웨이포인트(WP) 8에서 WP9로 이동 시, Z축 고도를 20m에서 6m로 낮춰야 합니다. 하지만 PX4 펌웨어의 내장된 최대 수직 강하 속도 제한(`MPC_Z_VEL_MAX_DN`)으로 인해 하강 속도가 매우 느립니다. 대회 규정상 PX4 파라미터 수정은 불가능하므로, ROS 2 Offboard 제어만으로 이 문제를 해결해야 합니다.

### 1.2. 목표 (Goal)
PX4의 안정적인 수평 위치 제어는 유지하면서, Z축 추력(Thrust)만을 직접 제어하여 WP8에서 WP9까지의 수직 강하 시간을 대폭 단축시킵니다. 이 과정에서 기체의 안정성을 해치지 않고, 목표 고도에 도달하면 다시 PX4의 완전한 위치 제어 모드로 안전하게 복귀해야 합니다.

## 2. 기술적 접근 방안 (Technical Approach)

### 2.1. 핵심 전략
목표 지점(WP9)의 수평 좌표(X, Y) 상공에서, 고도(Z)만을 PID 제어기를 통해 목표 고도(6m)까지 빠르게 낮춥니다.

1.  **혼합 제어 모드 활용**: PX4 Offboard 모드 중 `position`과 `thrust_and_torque`를 동시에 `True`로 설정합니다. 이 모드에서 PX4는 수평(X, Y) 위치 제어를 담당하고, ROS 2 노드는 수직(Z) 추력을 직접 제어할 수 있습니다.
2.  **상태 머신 확장**: 기존 상태 머신에 `FAST_DESCENT`라는 새로운 상태를 추가하여, 고속 강하 로직을 분리하고 관리합니다.
3.  **PID 제어기 도입**: 현재 고도와 목표 고도 간의 오차를 바탕으로 최적의 Z축 추력 값을 실시간으로 계산하는 PID 제어기를 구현합니다.

### 2.2. 미션 시나리오 흐름
1.  **`MOVING_TO_WAYPOINT` (WP8으로 이동)**: 드론은 기존의 위치 제어 방식을 사용하여 WP8 (고도 20m)에 접근합니다.
2.  **상태 전환**: 드론이 WP8에 도착하면, 미션 노드의 상태를 `FAST_DESCENT`로 전환합니다.
3.  **`FAST_DESCENT` (고속 강하 실행)**:
    *   수평 목표는 WP9의 (X, Y) 좌표로 고정시킵니다.
    *   수직 목표는 WP9의 고도(6m)로 설정합니다.
    *   PID 제어기가 현재 고도와 목표 고도(6m)를 기반으로 Z축 추력 값을 계산하여 매 루프마다 발행합니다.
    *   드론은 수평 위치를 유지하며 수직으로 빠르게 하강합니다.
4.  **제어 복귀**: 현재 고도가 목표 고도(6m)의 허용 오차 범위 내로 진입하면, PID 제어를 종료하고 다시 완전한 `MOVING_TO_WAYPOINT` (위치 제어) 상태로 복귀하여 WP9에 정밀하게 위치를 잡습니다.

## 3. 구현 상세 (Implementation Details)

### 3.1. `drone_control_utils.py` 수정

#### 3.1.1. `publish_offboard_control_mode` 함수 수정
`thrust_and_torque` 인자를 추가하여 혼합 제어 모드를 활성화할 수 있도록 수정합니다.

```python
def publish_offboard_control_mode(node, position=True, velocity=False, acceleration=False, attitude=False, body_rate=False, thrust_and_torque=False):
    msg = OffboardControlMode(
        position=position,
        # ...
        thrust_and_torque=thrust_and_torque,
        timestamp=int(node.get_clock().now().nanoseconds / 1000)
    )
    node.offboard_control_mode_publisher.publish(msg)
```

#### 3.1.2. `publish_thrust_setpoint` 신규 함수 추가
수평 위치 제어와 수직 추력 제어를 동시에 수행하기 위한 유틸리티 함수를 추가합니다.

```python
def publish_thrust_setpoint(node, thrust_value: float, horizontal_target_local_pos: list):
    """
    수평 위치는 PX4가 제어하고, 수직 추력만 직접 설정하여 발행합니다.
    """
    # 1. 혼합 제어 모드 설정 (수평: 위치, 수직: 추력)
    publish_offboard_control_mode(node, position=True, thrust_and_torque=True)

    # 2. Z축 추력 메시지 발행
    thrust_sp_msg = VehicleThrustSetpoint()
    thrust_sp_msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    thrust_sp_msg.xyz = [float('nan'), float('nan'), -np.clip(thrust_value, 0.0, 1.0)]
    node.thrust_setpoint_publisher.publish(thrust_sp_msg)

    # 3. 수평 위치 제어를 위한 TrajectorySetpoint 메시지 발행
    sp_msg = TrajectorySetpoint()
    sp_msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    # 수평(X,Y) 목표는 지정하고, 수직(Z) 목표는 비워둡니다.
    sp_msg.position = [horizontal_target_local_pos[0], horizontal_target_local_pos[1], float('nan')]
    sp_msg.velocity = [float('nan'), float('nan'), float('nan')]
    sp_msg.yaw = node.current_local_pos.heading # 현재 Yaw 유지
    node.trajectory_setpoint_publisher.publish(sp_msg)
```

### 3.2. `waypoint_mission.py` 수정

#### 3.2.1. `SimplePID` 클래스 추가
재사용 가능한 간단한 PID 제어기 클래스를 파일 상단에 추가합니다.

```python
class SimplePID:
    # (이전 답변에 제공된 PID 클래스 코드 전체)
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits):
        # ...
```

#### 3.2.2. `__init__` 함수 수정
PID 제어기를 멤버 변수로 초기화합니다.

```python
def __init__(self):
    super().__init__('waypoint_mission_node')
    # ...
    # 고속 강하용 PID 제어기 (게인은 초기값이며, 반드시 튜닝 필요)
    # 목표 고도: 6.0m, 출력(추력) 제한: 0.1(최소) ~ 0.45(호버링 직전)
    self.descent_pid = SimplePID(Kp=0.2, Ki=0.01, Kd=0.1, setpoint=6.0, output_limits=(0.1, 0.45))
```

#### 3.2.3. 상태 머신 로직 수정 (`run_mission_logic`, `_handle_...` 함수들)
-   `run_mission_logic`에 `FAST_DESCENT` 상태 분기 추가
-   `_handle_moving_to_waypoint_state`: WP8 도착 시 `self.state = "FAST_DESCENT"`로 전환하는 로직 추가
-   `_handle_fast_descent_state` 신규 함수 구현

```python
def _handle_fast_descent_state(self):
    """PID 제어기를 사용하여 추력을 조절하며 빠르게 하강하는 상태"""
    if self.current_map_pose is None or self.current_local_pos is None:
        return

    # 목표: WP9
    target_wp_index = 8 # WP9의 인덱스는 8
    target_wp_map = self.drone_waypoints[target_wp_index]
    current_altitude = self.current_map_pose.pose.position.z
    target_altitude = self.descent_pid.setpoint

    # 1. 목표 고도 도착 확인
    if abs(current_altitude - target_altitude) < 0.5: # 0.5m 이내로 진입
        self.get_logger().info("목표 고도 도달. 일반 위치 제어로 복귀합니다.")
        self.current_waypoint_index = 9 # 다음 미션 단계로 이동
        self.state = "MOVING_TO_WAYPOINT"
        return

    # 2. PID 제어기로 추력 계산
    now = self.get_clock().now()
    thrust_value = self.descent_pid.compute(current_altitude, now)

    # 3. 수평 목표(WP9의 XY)를 Local NED 좌표로 변환
    #    (dcu에 map->local 변환 함수가 이미 있으므로 활용)
    target_wp9_pose = PoseStamped()
    target_wp9_pose.pose.position.x = float(target_wp_map[0])
    target_wp9_pose.pose.position.y = float(target_wp_map[1])
    target_wp9_pose.pose.position.z = float(target_wp_map[2])
    
    target_ned_pos = dcu.convert_map_to_local_setpoint(
        self.current_local_pos, self.current_map_pose, target_wp9_pose
    )

    # 4. 계산된 추력과 수평 목표로 명령 발행
    dcu.publish_thrust_setpoint(self, thrust_value, target_ned_pos)

    self.get_logger().info(f"고속 강하 중... 현재 고도: {current_altitude:.2f}m, 목표 추력: {thrust_value:.3f}",
                         throttle_duration_sec=0.5)

```

## 4. 튜닝 및 검증 (Tuning & Verification)

### 4.1. 핵심 튜닝 파라미터
-   **PID Gains (`Kp`, `Ki`, `Kd`)**: 강하 프로파일을 결정하는 가장 중요한 값. `Kp` -> `Kd` -> `Ki` 순서로 튜닝 권장.
-   **Thrust Limits (`output_limits`)**: 최소/최대 추력을 제한하여 안전성 확보. 최대값은 호버링 추력(약 0.5)보다 낮아야 함.
-   **Altitude Tolerance**: 제어 모드 복귀 시점을 결정하는 고도 오차 허용 범위.

### 4.2. 검증 항목 (Acceptance Criteria)
1.  [ ] 드론은 WP8 도착 후 `FAST_DESCENT` 상태로 정상적으로 전환되는가?
2.  [ ] 강하 중 드론의 수평 위치가 WP9의 (X,Y) 상공에서 안정적으로 유지되는가?
3.  [ ] 강하 속도가 기존 위치 제어 방식 대비 유의미하게 빠른가?
4.  [ ] 목표 고도(6m) 근처에서 오버슛(overshoot) 없이 부드럽게 감속하는가?
5.  [ ] 목표 고도 도달 후, 일반 위치 제어 모드로 안전하게 복귀하는가?

## 5. 리스크 및 완화 방안 (Risks & Mitigations)

-   **리스크 1**: PID 게인 튜닝 실패로 인한 제어 불안정 (진동, 발산, 추락).
    -   **완화 방안**: **반드시 시뮬레이션에서 충분히 테스트한다.** `Kp` 값을 매우 낮은 값에서부터 점진적으로 올리며 테스트하고, 최소/최대 추력 제한을 보수적으로 설정한다.
-   **리스크 2**: 제어 모드 전환 시 기체의 갑작스러운 움직임.
    -   **완화 방안**: 상태 전환 전후로 PID 제어기의 누적 값(Integral)을 리셋하고, 부드러운 전환을 위한 로직을 추가할 수 있다.
---

# **[기능 개발 PRD] PID 제어를 이용한 고속 수직 강하 기능 (v1.1)**

*   **문서 버전:** 1.1 (Interactive Mission 통합 방안 추가)
*   **작성일:** 2025년 6월 26일
*   **담당:** Gemini (AI Assistant)
*   **상태:** 실행 계획 (Action Plan)

---

## 1. 개요 (Overview)

(이전 PRD와 동일)

## 2. 기술적 접근 방안 (Technical Approach)

(이전 PRD와 동일)

## 3. 구현 상세 (Implementation Details)

### 3.1. `drone_control_utils.py` 수정

(이전 PRD와 동일)

### 3.2. `waypoint_mission.py` 수정

(이전 PRD와 동일)

### **3.3. `interactive_mission.py` 수정 (테스트 및 튜닝 환경)**

`interactive_mission.py`에 `fall <고도>` 명령어를 추가하여 개발 중인 PID 고속 강하 기능을 독립적으로 테스트하고 튜닝할 수 있는 환경을 구축합니다.

#### 3.3.1. `__init__` 함수 수정
PID 제어기 및 관련 상태 변수를 `InteractiveMissionNode` 클래스에 추가합니다.

```python
# interactive_mission.py의 InteractiveMissionNode 클래스 __init__ 함수에 추가

# ... 기존 __init__ 내용 ...
# 고속 강하 테스트를 위한 PID 제어기 및 관련 변수
self.descent_pid = None # fall 명령 시 동적으로 생성
self.is_falling = False # PID 하강 중인지 여부를 나타내는 플래그
```

#### 3.3.2. `command_input_loop` 함수 수정
새로운 명령어 `fall`을 처리하는 로직을 추가합니다.

```python
# interactive_mission.py의 command_input_loop 함수 내에 추가

# ... 기존 명령어 처리 로직 ...
            elif command == "fall" and len(cmd) > 1:
                self._handle_fall_command(cmd[1])
            else:
                self.get_logger().warn(f"알 수 없는 명령: '{line.strip()}'")

# 콘솔 도움말에도 'fall' 명령어 추가
print("    fall <altitude>          - 현재 위치에서 지정한 절대 고도까지 PID 제어로 고속 강하")
```

#### 3.3.3. `_handle_fall_command` 신규 핸들러 함수 추가
`fall` 명령을 받았을 때 PID 제어기를 설정하고 `FAST_DESCENT` 상태로 전환하는 함수를 구현합니다.

```python
# interactive_mission.py에 새로운 핸들러 함수 추가

# SimplePID 클래스를 interactive_mission.py 상단에도 복사/붙여넣기 또는
# 별도의 유틸리티 파일로 분리하여 import 해야 합니다.
class SimplePID:
    # ... (이전 PRD의 PID 클래스 코드) ...

# ...

    def _handle_fall_command(self, altitude_str):
        """'fall' 명령을 처리하여 PID 고속 강하를 시작합니다."""
        if self.state not in ["IDLE"]:
            self.get_logger().warn(f"고속 강하를 시작할 수 없는 상태입니다. 먼저 'stop'으로 호버링하세요. 현재 상태: {self.state}")
            return
        
        try:
            target_altitude = float(altitude_str)
            current_altitude = self.current_map_pose.pose.position.z

            if target_altitude >= current_altitude:
                self.get_logger().error(f"목표 고도({target_altitude:.1f}m)는 현재 고도({current_altitude:.1f}m)보다 낮아야 합니다.")
                return

            self.get_logger().info(f"사용자 명령: FALL to {target_altitude:.1f}m. PID 제어 고속 강하를 시작합니다.")

            # PID 제어기 인스턴스 생성 (게인 값은 튜닝 필요!)
            self.descent_pid = SimplePID(
                Kp=0.2, Ki=0.01, Kd=0.1,
                setpoint=target_altitude,
                output_limits=(0.1, 0.45) # 호버링 추력(0.5)보다 낮은 범위
            )

            # 현재 위치를 목표 위치로 설정 (수평 위치 고정용)
            self.target_pose_map = copy.deepcopy(self.current_map_pose)
            self.state = "FAST_DESCENT" # 새로운 상태로 전환

        except ValueError:
            self.get_logger().error(f"잘못된 고도 값입니다: {altitude_str}")

```

#### 3.3.4. `run_mission_logic` 함수 수정
`FAST_DESCENT` 상태를 처리하는 로직을 추가합니다. `waypoint_mission.py`에 추가했던 로직과 거의 동일하지만, `interactive_mission`의 구조에 맞게 수정합니다.

```python
# interactive_mission.py의 run_mission_logic 함수에 분기 추가

def run_mission_logic(self):
    # ...
    # 미션별 상태 처리
    if self.state == "TAKING_OFF":
        self._handle_takeoff_state()
    # ... (기존 상태들) ...
    elif self.state == "FAST_DESCENT":
        self._handle_fast_descent_state() # 새로운 상태 처리기 호출
    elif self.state == "LANDING":
        self._handle_landing_state()

```

#### 3.3.5. `_handle_fast_descent_state` 신규 상태 처리 함수 추가
실제로 PID 제어를 수행하는 상태 처리 함수를 구현합니다.

```python
# interactive_mission.py에 새로운 상태 처리 함수 추가

def _handle_fast_descent_state(self):
    """PID 제어기를 사용하여 추력을 조절하며 빠르게 하강하는 상태 (대화형)"""
    if self.current_map_pose is None or self.current_local_pos is None or self.descent_pid is None:
        self.get_logger().warn("고속 강하에 필요한 정보가 부족하여 IDLE 상태로 복귀합니다.")
        self.state = "IDLE"
        return

    current_altitude = self.current_map_pose.pose.position.z
    target_altitude = self.descent_pid.setpoint

    # 1. 목표 고도 도착 확인
    if abs(current_altitude - target_altitude) < 0.5: # 0.5m 이내로 진입
        self.get_logger().info("목표 고도 도달. PID 제어 종료. IDLE 상태로 복귀합니다.")
        self.descent_pid = None # PID 컨트롤러 리셋
        self.target_pose_map = copy.deepcopy(self.current_map_pose) # 현재 위치에서 호버링하도록 목표 업데이트
        self.state = "IDLE"
        return

    # 2. PID 제어기로 추력 계산
    now = self.get_clock().now()
    thrust_value = self.descent_pid.compute(current_altitude, now)

    # 3. 수평 목표(현재 X,Y)를 Local NED 좌표로 변환
    target_ned_pos = dcu.convert_map_to_local_setpoint(
        self.current_local_pos, self.current_map_pose, self.target_pose_map
    )
    
    # 4. 계산된 추력과 수평 목표로 명령 발행
    dcu.publish_thrust_setpoint(self, thrust_value, target_ned_pos)

    self.get_logger().info(f"고속 강하 중... 현재 고도: {current_altitude:.2f}m, 목표 추력: {thrust_value:.3f}",
                         throttle_duration_sec=0.5)
```

## 4. 튜닝 및 검증 (Tuning & Verification)

### 4.1. `interactive_mission`을 이용한 튜닝 시나리오
1.  `ros2 run ... interactive_mission` 노드를 실행합니다.
2.  `start` -> `takeoff` 명령으로 드론을 이륙시킵니다.
3.  `go_wp 8` 명령으로 WP8 (고도 20m) 위치로 이동시킵니다.
4.  드론이 WP8에 도착하여 `IDLE` 상태가 되면, `fall 6` 명령을 입력합니다.
5.  콘솔 로그와 Gazebo 시뮬레이션을 통해 드론의 강하 프로파일, 최종 고도, 안정성 등을 관찰합니다.
6.  `Kp`, `Ki`, `Kd` 및 `output_limits` 값을 코드에서 수정한 뒤, 다시 컴파일하고 1~5단계를 반복하며 최적의 값을 찾습니다.

### 4.2. 검증 항목 (Acceptance Criteria)
(이전 PRD와 동일)

## 5. 리스크 및 완화 방안 (Risks & Mitigations)
(이전 PRD와 동일)