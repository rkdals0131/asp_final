# asp_ros2_ws

`asp_ros2_ws`는 자율 주행 및 로봇 제어를 위한 ROS 2 워크스페이스입니다. 이 워크스페이스는 컴퓨터 비전, UGV 경로 추종, 다중 객체 추적, PX4 연동 및 Gazebo 시뮬레이션 환경 설정과 관련된 패키지들을 포함하고 있습니다.

## 주요 패키지 및 스크립트 구조

asp_ros2_ws/src/
├── vision_opencv/
│   ├── cv_bridge/
│   ├── image_geometry/
│   └── opencv_tests/
├── ugv_controller/
│   ├── src/
│   │   └── path_follower_node.cpp
│   ├── launch/
│   │   └── path_follower.launch.py
│   └── Data/
├── multi_tracker/
│   ├── src/
│   │   └── multi_tracker_node.cpp
│   └── launch/
├── px4_ros_com/
│   ├── src/
│   └── launch/
├── px4_msgs/
│   ├── msg/
│   └── srv/
└── gazebo_env_setup/
├── src/
│   └── pose_tf_broadcaster.cpp
└── launch/

---

## 패키지별 상세 설명

### 1. `vision_opencv` 📦

ROS 2와 컴퓨터 비전 라이브러리인 OpenCV를 연동하는 역할을 합니다.

-   **`cv_bridge`**: ROS 2의 이미지 메시지(`sensor_msgs/Image`)와 OpenCV의 이미지 형식(`cv::Mat`) 간의 변환을 담당합니다.
-   **`image_geometry`**: 핀홀 및 스테레오 카메라 모델을 사용하여 3D 좌표와 2D 픽셀 간의 변환을 처리합니다.
-   **`opencv_tests`**: `cv_bridge`와 `image_geometry`의 기능을 검증하기 위한 테스트 패키지입니다.

### 2. `ugv_controller` 🚗

지상 로봇(UGV)의 경로 추종을 위한 컨트롤러 패키지입니다.

-   **`path_follower_node.cpp`**: UGV의 주행을 제어하는 메인 노드입니다.
    -   `Data` 폴더의 CSV 파일에서 주행 경로를 읽어옵니다.
    -   로봇의 현재 위치와 자세 정보를 구독합니다.
    -   PID 제어기를 사용하여 속도 명령을 발행합니다.
-   **`path_follower.launch.py`**: `path_follower_node`를 실행하고 파라미터를 설정하는 런치 파일입니다.
-   **`Data/*.csv`**: 직선, S자, 원형 등 다양한 주행 경로 좌표가 저장된 파일들입니다.

### 3. `multi_tracker` 🎯

ArUco 마커를 이용한 다중 객체 추적 패키지입니다.

-   **`multi_tracker_node.cpp`**: 카메라 이미지에서 ArUco 마커를 탐지하고, 마커의 상대 위치(pose)를 계산하여 발행하는 노드입니다.
-   **`x1_aruco_detector.launch.py` / `x500_aruco_detector.launch.py`**: 각기 다른 차량에 맞게 `multi_tracker_node`를 실행하는 런치 파일들입니다.

### 4. `px4_ros_com` & `px4_msgs` 🛰️

ROS 2와 PX4 오토파일럿 간의 통신을 위한 패키지입니다.

-   **`px4_ros_com`**: PX4의 오프보드(Offboard) 모드 제어 및 센서 데이터 수신 예제를 포함합니다.
-   **`px4_msgs`**: PX4의 uORB 메시지에 해당하는 ROS 2 메시지 정의를 담고 있습니다.

### 5. `gazebo_env_setup` 🌐

Gazebo 시뮬레이션 환경 설정을 위한 패키지입니다.

-   **`pose_tf_broadcaster.cpp`**: Gazebo의 모델 pose를 ROS 2의 TF(Transform)로 변환하여 `rviz2` 등에서 시각화할 수 있도록 합니다.
-   **`topic_bridge.launch.py`**: Gazebo 시뮬레이터와 ROS 2 간의 토픽을 연결하는 `ros_gz_bridge`를 실행합니다.

