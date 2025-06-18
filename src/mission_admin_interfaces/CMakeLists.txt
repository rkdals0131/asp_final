cmake_minimum_required(VERSION 3.15)
project(mission_admin_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지/서비스 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MissionComplete.srv"
  DEPENDENCIES std_msgs
)

# 런타임에 서비스 인터페이스를 사용할 패키지임을 알림
ament_export_dependencies(rosidl_default_runtime)

# 패키지 그룹 설정
# (package.xml의 <member_of_group>rosidl_interface_packages</member_of_group>와 쌍을 이룸)
ament_package()
