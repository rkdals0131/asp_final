#!/usr/bin/env python3
"""
드론 제어 관련 유틸리티 함수 모음
PX4 오프보드 제어, 짐벌 제어, 좌표 변환 등의 공통 기능을 제공합니다.
"""

import math
import rclpy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped


# === 각도 변환 유틸리티 함수들 ===

def degrees_to_radians(degrees):
    """
    도(degree)를 라디안으로 변환합니다.
    
    Args:
        degrees: 각도 (도 단위)
        
    Returns:
        float: 라디안 값
    """
    return math.radians(degrees)


def radians_to_degrees(radians):
    """
    라디안을 도(degree)로 변환합니다.
    
    Args:
        radians: 각도 (라디안 단위)
        
    Returns:
        float: 도 값
    """
    return math.degrees(radians)


def normalize_angle_degrees(angle_deg):
    """
    각도를 -180도에서 180도 범위로 정규화합니다.
    
    Args:
        angle_deg: 정규화할 각도 (도 단위)
        
    Returns:
        float: 정규화된 각도 (-180 ~ 180도)
    """
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def normalize_angle_radians(angle_rad):
    """
    각도를 -π에서 π 범위로 정규화합니다.
    
    Args:
        angle_rad: 정규화할 각도 (라디안 단위)
        
    Returns:
        float: 정규화된 각도 (-π ~ π)
    """
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def map_yaw_to_px4_yaw_degrees(map_yaw_deg):
    """
    맵 좌표계 yaw(X축 기준 0도)를 PX4 yaw(북쪽 기준 0도)로 변환합니다.
    
    Args:
        map_yaw_deg: 맵 좌표계 yaw 각도 (도 단위, X축이 0도)
        
    Returns:
        float: PX4 yaw 각도 (도 단위, 북쪽이 0도, -180~180도 범위)
    """
    # 맵 좌표계: X축(East)이 0도, 반시계방향이 양수
    # PX4 좌표계: 북쪽(North)이 0도, 시계방향이 양수
    px4_yaw_deg = 90.0 - map_yaw_deg
    return normalize_angle_degrees(px4_yaw_deg)


def px4_yaw_to_map_yaw_degrees(px4_yaw_deg):
    """
    PX4 yaw(북쪽 기준 0도)를 맵 좌표계 yaw(X축 기준 0도)로 변환합니다.
    
    Args:
        px4_yaw_deg: PX4 yaw 각도 (도 단위, 북쪽이 0도)
        
    Returns:
        float: 맵 좌표계 yaw 각도 (도 단위, X축이 0도, -180~180도 범위)
    """
    map_yaw_deg = 90.0 - px4_yaw_deg
    return normalize_angle_degrees(map_yaw_deg)


def publish_vehicle_command(node, command, **kwargs):
    """
    지정한 VehicleCommand를 퍼블리시하는 유틸리티 함수.
    
    Args:
        node: ROS2 노드 인스턴스
        command: VehicleCommand 타입 (예: VehicleCommand.VEHICLE_CMD_NAV_LAND)
        **kwargs: param1~param7 매개변수들
    """
    msg = VehicleCommand(
        command=command,
        timestamp=int(node.get_clock().now().nanoseconds / 1000),
        from_external=True,
        target_system=1,
        target_component=1
    )
    
    # param1~param7 설정
    for i in range(1, 8):
        param_value = float(kwargs.get(f"param{i}", 0.0))
        msg.__setattr__(f'param{i}', param_value)
    
    node.vehicle_command_publisher.publish(msg)


def publish_offboard_control_mode(node):
    """
    Offboard 제어 모드 메시지를 퍼블리시합니다.
    Position 제어 모드로 설정됩니다.
    """
    msg = OffboardControlMode(
        position=True,
        velocity=False,
        acceleration=False,
        attitude=False,
        timestamp=int(node.get_clock().now().nanoseconds / 1000)
    )
    node.offboard_control_mode_publisher.publish(msg)


def point_gimbal_at_target(node, drone_map_pose: PoseStamped, target_enu_pos: list):
    """
    드론의 현재 위치를 기준으로 ENU 좌표계의 목표 지점을 바라보도록 짐벌을 제어합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        drone_map_pose: 드론의 현재 map 좌표계 위치
        target_enu_pos: 목표 지점의 ENU 좌표 [x, y, z]
    """
    if drone_map_pose is None:
        node.get_logger().warn("Drone pose not available for gimbal control.", throttle_duration_sec=2.0)
        return

    drone_pos = drone_map_pose.pose.position
    
    # ENU 좌표계에서 목표 지점까지의 벡터 계산
    delta_x = target_enu_pos[0] - drone_pos.x  # East
    delta_y = target_enu_pos[1] - drone_pos.y  # North  
    delta_z = target_enu_pos[2] - drone_pos.z  # Up

    # Pitch 계산 (수평거리 기준)
    distance_2d = math.sqrt(delta_x**2 + delta_y**2)
    pitch_rad = math.atan2(delta_z, distance_2d)
    
    # Yaw 계산 (ENU를 PX4 북쪽 기준으로 변환)
    map_yaw_rad = math.atan2(delta_y, delta_x)  # ENU 기준
    map_yaw_deg = math.degrees(map_yaw_rad)
    px4_yaw_deg = 90.0 - map_yaw_deg  # PX4 북쪽 기준으로 변환
    
    # ±180도 범위로 정규화
    if px4_yaw_deg > 180.0:
        px4_yaw_deg -= 360.0
    if px4_yaw_deg < -180.0:
        px4_yaw_deg += 360.0

    # 짐벌 제어 명령 전송
    publish_vehicle_command(
        node,
        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
        param1=math.degrees(pitch_rad),  # Pitch
        param3=px4_yaw_deg,              # Yaw
        param7=2.0                       # MAV_MOUNT_MODE_YAW_BODY
    )


def convert_map_to_local_setpoint(current_local_pos, current_map_pose, target_map_pose):
    """
    목표 map 좌표를 현재 드론 위치 기준의 local NED 세트포인트로 변환합니다.
    
    Args:
        current_local_pos: 현재 드론의 local NED 위치
        current_map_pose: 현재 드론의 map 좌표계 위치
        target_map_pose: 목표 지점의 map 좌표계 위치
        
    Returns:
        list: [target_ned_x, target_ned_y, target_ned_z] local NED 좌표
    """
    # Map 좌표계에서의 변위 계산
    delta_map_x = target_map_pose.pose.position.x - current_map_pose.pose.position.x
    delta_map_y = target_map_pose.pose.position.y - current_map_pose.pose.position.y
    delta_map_z = target_map_pose.pose.position.z - current_map_pose.pose.position.z
    
    # ENU to NED 변환 적용
    delta_ned_x = delta_map_y   # North = ENU_Y
    delta_ned_y = delta_map_x   # East = ENU_X  
    delta_ned_z = -delta_map_z  # Down = -ENU_Z
    
    # 현재 local 위치에 변위 적용
    target_ned_x = current_local_pos.x + delta_ned_x
    target_ned_y = current_local_pos.y + delta_ned_y
    target_ned_z = current_local_pos.z + delta_ned_z
    
    return [float(target_ned_x), float(target_ned_y), float(target_ned_z)]


def publish_position_setpoint(node, current_local_pos, current_map_pose, target_map_pos, target_yaw_deg=None):
    """
    Map 좌표계 목표 위치를 기반으로 TrajectorySetpoint를 퍼블리시합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        current_local_pos: 현재 드론의 local 위치
        current_map_pose: 현재 드론의 map 위치
        target_map_pos: 목표 지점의 map 좌표 [x, y, z]
        target_yaw_deg: 목표 yaw 각도 (도 단위, 맵 좌표계 기준, None이면 yaw 제어 안함)
    """
    if current_map_pose is None or current_local_pos is None:
        return
    
    # 임시 target_map_pose 생성
    target_map_pose = PoseStamped()
    target_map_pose.pose.position.x = target_map_pos[0]
    target_map_pose.pose.position.y = target_map_pos[1] 
    target_map_pose.pose.position.z = target_map_pos[2]
    
    # Local NED 좌표로 변환
    target_ned = convert_map_to_local_setpoint(current_local_pos, current_map_pose, target_map_pose)
    
    # Yaw 처리
    yaw_value = math.nan  # 기본값: yaw 제어 안함
    if target_yaw_deg is not None:
        # 맵 좌표계 yaw를 PX4 yaw로 변환하고 라디안으로 변환
        px4_yaw_deg = map_yaw_to_px4_yaw_degrees(target_yaw_deg)
        yaw_value = degrees_to_radians(px4_yaw_deg)
    
    # TrajectorySetpoint 메시지 생성 및 퍼블리시
    sp_msg = TrajectorySetpoint(
        position=target_ned,
        yaw=yaw_value,
        timestamp=int(node.get_clock().now().nanoseconds / 1000)
    )
    node.trajectory_setpoint_publisher.publish(sp_msg)


def publish_position_setpoint_with_yaw(node, current_local_pos, current_map_pose, target_map_pos, target_yaw_deg):
    """
    Yaw 제어가 포함된 position setpoint를 퍼블리시합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        current_local_pos: 현재 드론의 local 위치
        current_map_pose: 현재 드론의 map 위치
        target_map_pos: 목표 지점의 map 좌표 [x, y, z]
        target_yaw_deg: 목표 yaw 각도 (도 단위, 맵 좌표계 기준)
    """
    publish_position_setpoint(node, current_local_pos, current_map_pose, target_map_pos, target_yaw_deg)


def check_arrival(current_map_pose, target_pos, tolerance=2.0):
    """
    목표 지점에 도착했는지 확인합니다.
    
    Args:
        current_map_pose: 현재 드론의 map 위치
        target_pos: 목표 지점 좌표 [x, y, z] 또는 PoseStamped
        tolerance: 도착 판정 허용 오차 (미터)
        
    Returns:
        bool: 도착 여부
    """
    if current_map_pose is None:
        return False
    
    pos = current_map_pose.pose.position
    
    # target_pos가 list인지 PoseStamped인지 확인
    if isinstance(target_pos, list):
        target_x, target_y, target_z = target_pos
    else:
        target_x = target_pos.pose.position.x
        target_y = target_pos.pose.position.y
        target_z = target_pos.pose.position.z
    
    # 3D 거리 계산
    distance = math.sqrt(
        (pos.x - target_x)**2 + 
        (pos.y - target_y)**2 + 
        (pos.z - target_z)**2
    )
    
    return distance < tolerance


def arm_and_offboard(node):
    """
    드론을 ARM하고 Offboard 모드로 전환합니다.
    """
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)


def land_drone(node):
    """
    드론 착륙 명령을 전송합니다.
    """
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_NAV_LAND)


def reset_gimbal(node):
    """
    짐벌을 정면(forward) 방향으로 리셋합니다.
    """
    publish_vehicle_command(
        node,
        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
        param1=0.0,  # Pitch = 0
        param3=0.0,  # Yaw = 0
        param7=2.0   # MAV_MOUNT_MODE_YAW_BODY
    )


def point_gimbal_down(node):
    """
    짐벌을 수직 아래 방향으로 향하게 합니다.
    """
    publish_vehicle_command(
        node,
        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
        param1=-90.0,  # Pitch = -90도 (아래)
        param3=0.0,    # Yaw = 0
        param7=2.0     # MAV_MOUNT_MODE_YAW_BODY
    ) 