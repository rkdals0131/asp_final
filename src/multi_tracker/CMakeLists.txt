cmake_minimum_required(VERSION 3.18)
project(multi_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 설치된 경로로 설정
set(HOME_DIR $ENV{HOME})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# OpenCV 컴포넌트 명시적 설정 (ArUco 포함)
find_package(OpenCV REQUIRED 
  COMPONENTS 
    core
    imgproc
    imgcodecs
    aruco
    calib3d
)

find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# include 디렉토리 설정
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 실행 파일 추가
add_executable(
  abscoord_marker_detection src/abscoord_marker_detection.cpp
)

# 필요한 의존성 링크
ament_target_dependencies(abscoord_marker_detection
rclcpp
sensor_msgs
geometry_msgs
std_msgs
OpenCV
cv_bridge
Eigen3
image_transport
tf2
tf2_geometry_msgs
vision_msgs
)

# 런치 설정
install(TARGETS 
  abscoord_marker_detection
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  multi_tracker/marker_stack.py
  multi_tracker/debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
