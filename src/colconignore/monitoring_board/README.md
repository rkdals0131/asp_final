# ASP Mission Control Dashboard v3.0

## 개요

이 패키지는 드론(UAV)과 무인차량(UGV)의 통합 미션을 관리하는 실시간 미션 컨트롤 대시보드를 제공합니다. curses 라이브러리를 사용한 터미널 기반 UI로 실시간 모니터링과 미션 제어가 가능합니다.

## 주요 기능

### 🎯 미션 시퀀스 관리
- UGV의 드론 이륙 지점 이동
- 드론의 자동 ARM 및 이륙
- 동기화된 미션 진행
- 랑데뷰 지점에서의 최종 호버링

### 📡 실시간 모니터링
- 시스템 상태 (Pre-flight Check)
- 플랫폼 원격측정 (위치, 속도, 상태)
- 미션 페이로드 (마커 검출)
- 미션 진행 상황

### 🎮 대화식 제어
- 키보드를 통한 실시간 미션 제어
- 미션 시작/중단/리셋
- 직관적인 색상 코딩

## 미션 시나리오

1. **초기화**: 모든 시스템 상태 확인
2. **UGV 이동**: 이륙 지점으로 이동 (드론 ARM 금지)
3. **드론 ARM**: 이륙 지점 도착 후 드론 ARM
4. **드론 이륙**: 안전한 이륙 수행
5. **UGV 재개**: 드론 이륙 완료 후 UGV 미션 재개
6. **미션 진행**: 각 플랫폼의 웨이포인트 추종
7. **랑데뷰**: 최종 지점에서 호버링

## 설치 및 빌드

```bash
# 워크스페이스로 이동
cd ~/your_ros2_ws

# 빌드
colcon build --packages-select mission_admin_interfaces monitoring_board ugv_controller uav_controller

# 소스
source install/setup.bash
```

## 실행 방법

### 1. 전체 시스템 실행 (권장)
```bash
ros2 launch monitoring_board mission_control.launch.py
```

### 2. 개별 노드 실행
```bash
# 미션 컨트롤 대시보드
ros2 run monitoring_board mission_control

# UGV 컨트롤러
ros2 run ugv_controller path_follower_node

# 드론 컨트롤러
ros2 run uav_controller offboard_control_abscoord_traversal
```

## 대시보드 사용법

### 키보드 단축키
- **S**: 미션 시작 (UGV가 이륙 지점으로 이동)
- **A**: 미션 중단
- **R**: 미션 상태 리셋
- **Q**: 대시보드 종료

### 화면 구성
1. **헤더**: 시스템 정보 및 시간
2. **미션 상태**: 현재 미션 단계 및 경과 시간
3. **시스템 상태**: Pre-flight Check 결과
4. **플랫폼 원격측정**: 드론/차량 상태 정보
5. **미션 페이로드**: 마커 검출 결과
6. **컨트롤**: 사용 가능한 명령어

### 상태 색상
- 🟢 **녹색**: 정상/성공
- 🔴 **빨간색**: 오류/위험
- 🟡 **노란색**: 경고/대기
- 🔵 **파란색**: 정보
- 🟣 **자주색**: 미션 상태

## 미션 상태

| 상태 | 설명 |
|------|------|
| INIT | 시스템 초기화 중 |
| READY | 미션 시작 준비 완료 |
| UGV_TO_TAKEOFF | UGV가 이륙 지점으로 이동 중 |
| DRONE_ARMING | 드론 ARM 진행 중 |
| DRONE_TAKEOFF | 드론 이륙 중 |
| MISSION_ACTIVE | 미션 활성화 (양 플랫폼 이동) |
| DRONE_APPROACH | 드론 랑데뷰 지점 접근 |
| DRONE_HOVER | 드론 최종 호버링 |
| MISSION_COMPLETE | 미션 완료 |
| MISSION_ABORT | 미션 중단 |

## 서비스 인터페이스

### MissionComplete.srv
```
uint8 mission_id
---
bool success
```

### 미션 ID 정의
- 1: UGV 이륙 지점 도착
- 2: 드론 이륙 완료
- 3: UGV 미션 완료
- 4: 드론 랑데뷰 지점 도착
- 5: 드론 최종 호버링 완료

## 토픽 구조

### 퍼블리시
- `/mission_complete` (service): 미션 완료 신호
- `/ugv/mission_command`: UGV 명령
- `/drone/mission_command`: 드론 명령

### 서브스크라이브
- `/fmu/out/vehicle_local_position`: 드론 위치
- `/model/X1/odometry`: 차량 오도메트리
- `/marker_detections`: 마커 검출
- `/drone/state`: 드론 상태
- `/vehicle/state`: 차량 상태

## 문제 해결

### curses 관련 오류
```bash
# 터미널 설정 확인
echo $TERM

# 대체 터미널 사용
export TERM=xterm-256color
```

### 서비스 연결 실패
```bash
# 서비스 확인
ros2 service list | grep mission_complete

# 노드 상태 확인
ros2 node list
```

### TF 변환 오류
```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# TF 상태 확인
ros2 topic echo /tf
```

## 의존성

### ROS2 패키지
- `rclpy`
- `px4_msgs`
- `geometry_msgs`
- `std_msgs`
- `sensor_msgs`
- `nav_msgs`
- `tf2_ros`
- `vision_msgs`
- `mission_admin_interfaces`

### Python 패키지
- `curses` (시스템 기본 포함)
- `numpy`
- `math`
- `threading`
- `datetime`

## 라이선스

Apache License 2.0

## 개발자

ASP (Autonomous Systems & Perception) Lab
- 유지보수: user1 (kikiws70@gmail.com)

## 버전 이력

- v3.0: curses 기반 실시간 대시보드 및 미션 컨트롤 추가
- v2.2: 기본 모니터링 대시보드
- v1.0: 초기 버전 