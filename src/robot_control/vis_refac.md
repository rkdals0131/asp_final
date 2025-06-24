## 시각화 로직 리팩터링 최종 계획 (v2)

### 1. 핵심 목표
- 모든 **마커 생성(Creation) 로직**을 `utils/viz_factory.py` 파일 하나로 통합한다.
- 마커 **발행(Publishing) 책임**은 각 기능별 노드가 명확히 분담한다.
- Ground-Truth 마커 시각화와 실시간 탐지 마커 시각화를 **완전히 다른 노드로 분리**하여 단일 책임 원칙을 준수한다.

### 2. 파일 구조 변경
- **신규 생성:**
    - `robot_control/admin/ground_truth_visualizer.py`: Ground-Truth CSV를 읽어 RViz에 마커를 발행하는 전용 노드.
- **수정/이동:**
    - `robot_control/utils/marker_visual.py` -> `robot_control/uav/detected_marker_visualizer.py`: 실시간 마커 탐지 시각화 및 CSV 저장 전용 노드로 역할 축소.
    - `robot_control/ugv/path_follower_node.py`: 내부 시각화 코드를 `viz_factory` 호출로 변경 (기존 계획 유지).
    - `setup.py`: `ground_truth_visualizer`를 포함한 모든 신규/변경된 노드를 `entry_points`에 정확히 등록.
- **삭제:**
    - `robot_control/utils/visualization_utils.py` (기능 이전 완료 후)

### 3. 세부 설계
- **`utils/viz_factory.py`:**
    - 상태 없는(stateless) 마커 생성 함수 모음 (기존 계획 유지).
- **`uav/detected_marker_visualizer.py`:**
    - `/marker_detections` 토픽만 구독.
    - **Ground-Truth 관련 모든 코드 제거.**
    - `viz_factory.create_detected_markers`를 호출하여 `/detected_markers_viz` 토픽으로 마커 발행.
    - CSV 저장 기능 유지.
- **`admin/ground_truth_visualizer.py`:**
    - 파라미터로 받은 CSV 파일 경로에서 Ground-Truth 마커 정보를 로드.
    - `viz_factory.create_ground_truth_markers`를 호출하여 마커 생성.
    - 생성된 마커를 `/ground_truth_markers` 토픽에 **Latched QoS**로 발행하여 RViz에 항상 표시.

### 4. 단계별 실행 계획
1.  **(완료)** `vis_refac.md` 파일을 현재 계획으로 업데이트.
2.  `setup.py` 파일을 올바른 위치(`robot_control/setup.py`)에 정확한 내용으로 재생성.
3.  `admin/ground_truth_visualizer.py` 노드를 신규 작성.
4.  `uav/detected_marker_visualizer.py`에서 Ground-Truth 관련 로직을 완전히 제거.
5.  (기존 계획대로) `path_follower_node.py` 및 `visualization_utils.py` 리팩터링 및 삭제.
6.  `ros2 launch`를 통해 분리된 시각화 노드들이 정상 동작하는지 통합 테스트.

### 5. 데이터 흐름
```
PathFollower ▶ /viz/path & /viz/velocities
                │
                ▼
       mission_visual_node ─▶ /visualization_marker_array ─▶ RViz2
DetectionNode ▶ /marker_detections ┘
```

### 6. 단계별 작업
1단계: `viz_factory.py` 초안 작성 및 유닛테스트 (Marker 필드 검증)
2단계: `mission_visual_node.py` 골격 구현, 주요 토픽 수신·마커 생성 연결
3단계: `path_follower_node.py` 시각화 코드 제거 → 토픽 발행으로 교체
4단계: Ground-Truth/CSV 저장 기능 이전 후 `marker_visual.py` 제거
5단계: `visualization_utils.py` 내용 정리·삭제 여부 결정
6단계: 전체 통합 테스트 (RViz 확인 + ros2 bag 재생)

### 7. 주의사항
- **새로운 디렉터리 금지** → 헬퍼는 `utils/` 바로 아래에 저장
- Marker ID 충돌 방지: `mission_visual_node` 내부 전역 카운터 관리
- QoS: 모든 시각화 Topic `TRANSIENT_LOCAL` 적용하여 RViz 재접속 시 재표시
- 파라미터·프레임·컬러 등 하드코딩 값은 상단 상수화

### 8. 예상 일정(업무일 기준)
| Day | 작업 | 산출물 |
|---|---|---|
| 1 | viz_factory.py 구현 | 마커 생성 함수 모듈, 테스트 스크립트 |
| 2 | mission_visual_node.py V1 | 기본 Marker 출력 확인 |
| 3 | path_follower_node 리팩터링 | 기존 시각화 코드 제거 |
| 4 | 기능 이관 최종·불필요 파일 삭제 | marker_visual.py 폐기 |
| 5 | 문서화/리뷰 | README, 테스트 결과 |

### 9. 완료 기준 (Definition of Done)
- RViz에서 모든 Marker 레이어 확인 가능 (경로·속도·Lookahead·마커탐지)
- `ros2 run robot_control mission_visual_node` 단일 노드 실행으로 시각화 완결
- 기존 시각화 관련 소스 중복 70% 이상 감소
- 코드 스타일·테스트 모두 통과 