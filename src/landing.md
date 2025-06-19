## 🚁 정밀 착륙 미션 통합 개발 계획서 (v1.1)

### 1. 최종 목표

기존 웨이포인트 미션 수행 완료 후, UGV에 부착된 ArUco 마커를 실시간으로 추적하여 드론이 안정적으로 정밀 착륙(Precision Landing)하는 기능을 완성합니다.

### 2. 시나리오 분석

현재 미션은 최종 웨이포인트에 도달하면 `MISSION_COMPLETE_HOVER` 상태에 진입하여 무한정 호버링합니다. 이 로직을 정밀 착륙 시나리오로 확장합니다.

1.  **웨이포인트 미션**: 드론이 사전 설정된 웨이포인트와 응시 지점을 따라 비행합니다.
2.  **랑데부 지점 도착**: 마지막 웨이포인트(착륙 준비 지점)에 도착하여 2초간 호버링을 완료합니다.
3.  **착륙 모드 전환**:
    * `waypoint_mission_node`는 `MISSION_COMPLETE_HOVER` 상태 대신, 새로운 **`PRECISION_LANDING`** 상태로 전환합니다.
    * 전환 직후, `MissionComplete.srv` 서비스를 이용해 `mission_control_node`에 `DRONE_START_PRECISION_LANDING (ID=6)` 신호를 전송합니다.
4.  **마커 탐지 모드 지시**:
    * `mission_control_node`는 ID 6 수신 시, `multi_tracker_node`에 착륙용 마커(0.5m) 탐지를 지시하는 `"DETECT_LANDING_MARKER"` 메시지를 `/multi_tracker/command` 토픽으로 발행합니다.
5.  **UGV 도착 대기 및 하강 허가**:
    * UGV가 랑데부 지점에 도착하면 `UGV_RENDEZVOUS_ARRIVAL (ID=7)` 신호를 `mission_control_node`에 전송합니다.
    * `mission_control_node`는 드론에 `'ugv_arrived'` 명령을 전송하여 하강 허가를 내립니다.
    * UGV 도착 전까지는 드론이 마지막 웨이포인트에서 호버링하며 대기합니다.
6.  **단계별 정밀 제어 및 하강**:
    * **1단계 (마커 탐색)**: UGV 도착 후 마커 미탐지 시, 천천히 하강하며 마커를 탐색합니다.
    * **2단계 (수평 정렬)**: 마커 탐지 시 고도를 유지하며 마커 중앙으로 수평 이동합니다.
    * **3단계 (정밀 하강)**: 수평 정렬 완료 후 마커 중앙에서 천천히 하강합니다.
    * **4단계 (지속적 정밀 제어)**: 하강 중 정렬이 벗어나면 고도 유지하며 재정렬
7.  **착륙 완료**:
    * 드론의 고도와 수평 오차가 엄격한 임계값 이내로 진입하면, 최종 `LANDING` 상태로 전환하여 착륙을 완료합니다.

### 3. 식별된 문제점 및 해결 방안

1.  **`waypoint_mission_node`의 마커 정보 부재**: ✅ **해결됨** - 마커 위치 구독자 및 콜백 함수 구현 완료
2.  **모호한 Yaw 제어**: **보류** - 현재는 기본 위치 제어를 사용하며, 필요 시 추후 개선
3.  **마커 탐색 중 호버링 문제**: ✅ **해결됨** - UGV 도착 후 천천히 하강하며 마커 탐색하도록 변경
4.  **UGV-드론 동기화 부재**: ✅ **해결됨** - UGV 랑데부 도착 신호 및 하강 허가 메커니즘 추가

### 4. 패키지별 상세 개발 계획

#### 가. `robot_control` 패키지

##### 🔹 `uav/waypoint_mission.py` 수정 ✅ **완료**

* **구현 완료 항목**:
    1.  **`PRECISION_LANDING` 상태 추가**: 정밀 착륙 상태 및 관련 파라미터 추가
    2.  **마커 위치 구독자**: `/marker_detections` 토픽 구독 및 착륙 마커 필터링
    3.  **UGV 도착 신호 처리**: `'ugv_arrived'` 명령 수신 시 하강 허가 플래그 설정
    4.  **단계별 정밀 착륙 로직**:
        * UGV 미도착 시: 호버링 대기
        * 마커 미탐지 시: 천천히 하강하며 탐색 (`search_descent_speed: 0.15m/s`)
        * 마커 탐지 시: 4단계 지속적 정밀 제어
            * **수평 정렬**: 고도 유지하며 수평 정렬 (`horizontal_tolerance: 0.1m`)
            * **정밀 하강**: 정밀 정렬 상태에서만 하강 (`precision_horizontal_tolerance: 0.05m`)
            * **재정렬**: 하강 중 정렬이 벗어나면 고도 유지하며 재정렬
            * **최종 착륙**: 수평/수직 모든 조건 만족 시에만 `LANDING` 상태 전환

##### 🔹 `admin/mission_control_node.py` 수정 ✅ **완료**

* **구현 완료 항목**:
    1.  **Mission ID 추가**: `UGV_RENDEZVOUS_ARRIVAL: 7` 추가
    2.  **UGV 도착 플래그**: `ugv_arrived_at_rendezvous` 상태 관리
    3.  **드론 하강 허가 신호**: UGV 도착 시 드론에 `'ugv_arrived'` 명령 전송
    4.  **마커 탐지 지시**: 정밀 착륙 시작 시 `multi_tracker`에 `"DETECT_LANDING_MARKER"` 명령 전송

#### 나. `multi_tracker` 패키지 ✅ **완료**

##### 🔹 `src/abscoord_marker_detection.cpp` 및 `include/multi_tracker/abscoord_marker_detection.hpp` 수정

* **구현 완료 항목**:
    1.  **착륙 모드 플래그**: `_is_landing_mode` 변수로 마커 크기 제어
    2.  **명령 구독자**: `/multi_tracker/command` 토픽 구독
    3.  **조건부 마커 크기**: 착륙 모드 시 0.5m, 일반 모드 시 1.0m 사용
    4.  **빌드 오류 수정**: 사용하지 않는 함수 제거 및 CMake 설정 정정

### 5. 구현 상태 및 테스트 준비

#### ✅ 구현 완료 사항

1.  **코드 구현**: 모든 패키지의 정밀 착륙 기능 구현 완료
2.  **빌드 검증**: 컴파일 오류 및 경고 해결 완료
3.  **로직 검증**: 단계별 하강 및 정렬 알고리즘 구현 완료

#### 🧪 다음 테스트 단계

1.  **단위 테스트**: 각 노드별 기능 동작 확인
2.  **통합 테스트**: 전체 미션 시나리오 실행
3.  **마커 크기 전환 테스트**: 일반 마커(1.0m) → 착륙 마커(0.5m) 전환 검증
4.  **정밀 착륙 정확도 테스트**: 수평/수직 허용 오차 내 착륙 성공률 측정

### 6. 파라미터 설정 가이드

정밀 착륙 성능 조정을 위한 주요 파라미터:

```yaml
# waypoint_mission_node 파라미터
landing_altitude: 0.5                    # 최종 착륙 고도 (m)
descent_speed: 0.2                       # 정밀 하강 속도 (m/s)
search_descent_speed: 0.15               # 마커 탐색 중 하강 속도 (m/s)
horizontal_tolerance: 0.1                # 수평 정렬 허용 오차 (m)
precision_horizontal_tolerance: 0.05     # 정밀 하강 시 수평 허용 오차 (m)
vertical_tolerance: 0.2                  # 수직 허용 오차 (m)
landing_marker_id: 10                    # 착륙용 마커 ID

# multi_tracker_node 파라미터
marker_size: 1.0                         # 일반 마커 크기 (m)
landing_marker_size: 0.5                 # 착륙 마커 크기 (m)
```