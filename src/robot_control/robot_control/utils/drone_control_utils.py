#!/usr/bin/env python3
"""
드론 제어 관련 유틸리티 함수 모음
PX4 오프보드 제어, 짐벌 제어, 좌표 변환 등의 공통 기능을 제공
"""

import math
import rclpy
import time
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleAttitudeSetpoint, ActuatorMotors
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node





# ============================================
# 기존 유틸리티 함수들 (단순화)
# ============================================

# === 각도 변환 함수들 ===
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


def get_quaternion_from_euler(roll_rad, pitch_rad, yaw_rad):
    """
    오일러 각(라디안)을 쿼터니언으로 변환합니다.
    
    Args:
        roll_rad: Roll 각도 (라디안)
        pitch_rad: Pitch 각도 (라디안)
        yaw_rad: Yaw 각도 (라디안)
        
    Returns:
        list: [w, x, y, z] 쿼터니언
    """
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]


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


def publish_offboard_control_mode(node, position=True, velocity=False, acceleration=False, attitude=False, body_rate=False, thrust_and_torque=False, actuator=False):
    """
    Offboard 제어 모드 메시지를 퍼블리시
    Args:
        node: ROS2 노드 인스턴스
        position (bool): 위치 제어 활성화 여부
        velocity (bool): 속도 제어 활성화 여부
        acceleration (bool): 가속도 제어 활성화 여부
        attitude (bool): 자세 제어 활성화 여부
        body_rate (bool): 각속도 제어 활성화 여부
        thrust_and_torque (bool): 추력/토크 제어 활성화 여부
        actuator (bool): 액추에이터 직접 제어 활성화 여부
    """
    msg = OffboardControlMode(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        attitude=attitude,
        body_rate=body_rate,
        thrust_and_torque=thrust_and_torque,
        direct_actuator=actuator,  # 액추에이터 직접 제어 모드
        timestamp=int(node.get_clock().now().nanoseconds / 1000)
    )
    node.offboard_control_mode_publisher.publish(msg)


def publish_attitude_setpoint(node, q: list, thrust_value: float):
    """
    자세 및 추력 세트포인트를 퍼블리시합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        q: 목표 자세 쿼터니언 [w, x, y, z]
        thrust_value: 정규화된 추력 값 (0.0 ~ 1.0)
    """
    if not hasattr(node, 'attitude_setpoint_publisher'):
        node.get_logger().error("attitude_setpoint_publisher가 초기화되지 않았습니다!")
        return
        
    # 입력값 검증 및 제한
    thrust_clamped = max(0.0, min(1.0, float(thrust_value)))
    quaternion = [float(val) for val in q]
    
    msg = VehicleAttitudeSetpoint()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    msg.q_d = quaternion
    
    # PX4는 FRD (Front-Right-Down) 바디 프레임을 사용하므로,
    # 상승을 위한 추력은 -Z 방향으로 작용해야 함.
    # thrust_body[2]가 음수일 때 상승, 양수일 때 하강
    msg.thrust_body = [0.0, 0.0, -thrust_clamped]
    
    node.attitude_setpoint_publisher.publish(msg)
    
    # 디버깅을 위한 상세 로그 (필요시)
    # node.get_logger().debug(f"자세 명령 발행: q={quaternion}, thrust={thrust_clamped}")


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
    drone_q = drone_map_pose.pose.orientation

    # 드론의 현재 yaw 계산 (map 좌표계, ENU 기준)
    siny_cosp = 2 * (drone_q.w * drone_q.z + drone_q.x * drone_q.y)
    cosy_cosp = 1 - 2 * (drone_q.y * drone_q.y + drone_q.z * drone_q.z)
    drone_map_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    
    # ENU 좌표계에서 목표 지점까지의 벡터 계산
    delta_x = target_enu_pos[0] - drone_pos.x  # East
    delta_y = target_enu_pos[1] - drone_pos.y  # North  
    delta_z = target_enu_pos[2] - drone_pos.z  # Up

    # Pitch 계산 (수평거리 기준)
    distance_2d = math.sqrt(delta_x**2 + delta_y**2)
    pitch_rad = math.atan2(delta_z, distance_2d)
    
    # 목표지점의 절대 Yaw 계산 (map 좌표계 기준)
    target_map_yaw_rad = math.atan2(delta_y, delta_x)

    # 드론 yaw와 목표 yaw를 PX4 좌표계로 변환 (North=0, CW_positive)
    drone_px4_yaw_deg = map_yaw_to_px4_yaw_degrees(math.degrees(drone_map_yaw_rad))
    target_px4_yaw_deg = map_yaw_to_px4_yaw_degrees(math.degrees(target_map_yaw_rad))

    # 드론 기준 상대 yaw 계산
    relative_yaw_deg = target_px4_yaw_deg - drone_px4_yaw_deg

    # ±180도 범위로 정규화
    final_yaw_deg = normalize_angle_degrees(relative_yaw_deg)

    # 짐벌 제어 명령 전송
    publish_vehicle_command(
        node,
        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
        param1=math.degrees(pitch_rad),  # Pitch
        param3=final_yaw_deg,            # Yaw (body frame)
        param7=2.0                       # MAV_MOUNT_MODE_MAVLINK_TARGETING
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


def publish_position_setpoint(node, target_map_pos, target_yaw_deg=None):
    """
    Map 좌표계 목표 위치를 기반으로 TrajectorySetpoint를 퍼블리시 (위치 제어 전용)
    
    Args:
        node: ROS2 노드 인스턴스
        target_map_pos: 목표 지점의 map 좌표 [x, y, z]
        target_yaw_deg: 목표 yaw 각도 (도 단위, 맵 좌표계 기준, None이면 yaw 제어 안함)
    """
    publish_trajectory_setpoint(node, 
                                target_map_pos=target_map_pos, 
                                target_yaw_deg=target_yaw_deg)


def publish_trajectory_setpoint(node, target_map_pos=None, target_map_vel=None, target_yaw_deg=None):
    """
    Map 좌표계 기준 목표 위치/속도를 기반으로 TrajectorySetpoint을 퍼블리시.
    이 함수는 위치, 속도, yaw 제어를 위한 모든 로직을 통합.
    
    Args:
        node: ROS2 노드 인스턴스
        target_map_pos (list, optional): 목표 map 좌표 [x, y, z]
        target_map_vel (list, optional): 목표 map 속도 [vx, vy, vz]
        target_yaw_deg (float, optional): 목표 yaw 각도 (도 단위, 맵 좌표계 기준)
    """
    if node.current_map_pose is None or node.current_local_pos is None:
        node.get_logger().warn("위치 정보가 없어 setpoint를 발행할 수 없습니다.", throttle_duration_sec=2.0)
        return

    sp_msg = TrajectorySetpoint()
    sp_msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    
    # 제어 모드 결정
    use_pos = target_map_pos is not None
    use_vel = target_map_vel is not None
    publish_offboard_control_mode(node, position=use_pos, velocity=use_vel)

    # 위치 설정 (map -> local NED 변환)
    if use_pos:
        # 임시 target_map_pose 생성
        temp_target_map_pose = PoseStamped()
        temp_target_map_pose.pose.position.x = float(target_map_pos[0])
        temp_target_map_pose.pose.position.y = float(target_map_pos[1])
        temp_target_map_pose.pose.position.z = float(target_map_pos[2])
        
        target_ned_pos = convert_map_to_local_setpoint(
            node.current_local_pos, node.current_map_pose, temp_target_map_pose)
        
        sp_msg.position = target_ned_pos
    else:
        sp_msg.position = [float('nan'), float('nan'), float('nan')]

    # 속도 설정 (map(ENU) -> local NED 변환)
    if use_vel:
        # ENU to NED 변환 적용
        sp_msg.velocity[0] = float(target_map_vel[1])  # North = ENU_Y
        sp_msg.velocity[1] = float(target_map_vel[0])  # East  = ENU_X
        sp_msg.velocity[2] = float(-target_map_vel[2]) # Down  = -ENU_Z
    else:
        sp_msg.velocity = [float('nan'), float('nan'), float('nan')]

    # Yaw 설정 (map -> PX4 NED 변환)
    if target_yaw_deg is not None:
        px4_yaw_rad = degrees_to_radians(map_yaw_to_px4_yaw_degrees(target_yaw_deg))
        sp_msg.yaw = normalize_angle_radians(px4_yaw_rad)
    else:
        # Yaw 제어를 원하지 않을 경우, 현재 드론의 yaw를 유지하도록 설정
        # 이렇게 하면 위치/속도 제어 시 yaw가 의도치 않게 변경되는 것을 방지
        if node.current_local_pos and hasattr(node.current_local_pos, 'heading'):
             sp_msg.yaw = node.current_local_pos.heading
        else:
             sp_msg.yaw = float('nan')

    node.trajectory_setpoint_publisher.publish(sp_msg)


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
    
    # target_pos가 PoseStamped인 경우와 list인 경우 모두 처리
    if isinstance(target_pos, PoseStamped):
        target_point = target_pos.pose.position
    elif isinstance(target_pos, list) and len(target_pos) == 3:
        target_point = type('obj', (object,), {'x': target_pos[0], 'y': target_pos[1], 'z': target_pos[2]})()
    else:
        # 잘못된 타입의 target_pos가 들어오면 False 반환
        return False

    dx = current_map_pose.pose.position.x - target_point.x
    dy = current_map_pose.pose.position.y - target_point.y
    dz = current_map_pose.pose.position.z - target_point.z
    
    # 3D 거리 계산
    distance = math.sqrt(
        (dx)**2 + 
        (dy)**2 + 
        (dz)**2
    )
    
    return distance < tolerance


def arm_and_offboard(node):
    """
    드론을 ARM하고 Offboard 모드로 전환합니다.
    """
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)


def land_drone(node: Node):
    """드론에게 착륙 명령을 보냅니다."""
    node.get_logger().debug("LAND 명령 전송")
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_NAV_LAND)


def disarm_drone(node: Node):
    """드론에게 Disarm 명령을 보냅니다."""
    node.get_logger().debug("DISARM 명령 전송")
    publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)


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


def set_gimbal_angle(node, pitch_deg=0.0, yaw_deg=0.0, roll_deg=0.0):
    """
    짐벌을 지정한 각도로 설정합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        pitch_deg: Pitch 각도 (도 단위, -90~90)
        yaw_deg: Yaw 각도 (도 단위, -180~180, body frame 기준)
        roll_deg: Roll 각도 (도 단위, -180~180, 일반적으로 지원되지 않을 수 있음)
    """
    # 각도 제한 적용
    pitch_deg = max(-90.0, min(90.0, pitch_deg))
    yaw_deg = normalize_angle_degrees(yaw_deg)
    roll_deg = normalize_angle_degrees(roll_deg)
    
    publish_vehicle_command(
        node,
        VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
        param1=pitch_deg,  # Pitch
        param2=roll_deg,   # Roll (일반적으로 지원되지 않을 수 있음)
        param3=yaw_deg,    # Yaw (body frame)
        param7=2.0         # MAV_MOUNT_MODE_MAVLINK_TARGETING
    )


def enu_to_local_frame(map_pos, current_local_pos, current_map_pose):
    """
    Map(ENU) 좌표를 드론의 Local 프레임 좌표로 변환합니다.
    
    Args:
        map_pos: Map(ENU) 좌표 [x, y, z]
        current_local_pos: 현재 드론의 local NED 위치
        current_map_pose: 현재 드론의 map 좌표계 위치
        
    Returns:
        list: [target_ned_x, target_ned_y, target_ned_z] local NED 좌표
    """
    # Map 좌표계에서의 변위 계산
    delta_map_x = map_pos[0] - current_map_pose.pose.position.x
    delta_map_y = map_pos[1] - current_map_pose.pose.position.y
    delta_map_z = map_pos[2] - current_map_pose.pose.position.z
    
    # ENU to NED 변환 적용
    delta_ned_x = delta_map_y   # North = ENU_Y
    delta_ned_y = delta_map_x   # East = ENU_X  
    delta_ned_z = -delta_map_z  # Down = -ENU_Z
    
    # 현재 local 위치에 변위 적용
    target_ned_x = current_local_pos.x + delta_ned_x
    target_ned_y = current_local_pos.y + delta_ned_y
    target_ned_z = current_local_pos.z + delta_ned_z
    
    return [float(target_ned_x), float(target_ned_y), float(target_ned_z)]


def publish_actuator_motors(node, motor_outputs: list):
    """
    개별 모터 출력을 직접 제어합니다. (모든 PX4 안전장치 우회)
    
    Args:
        node: ROS2 노드 인스턴스
        motor_outputs: 각 모터의 출력 값 리스트 (0.0 ~ 1.0, 일반적으로 4개 모터)
                      [front_right, back_left, front_left, back_right] 순서
    """
    if not hasattr(node, 'actuator_motors_publisher'):
        node.get_logger().error("actuator_motors_publisher가 초기화되지 않았습니다!")
        return
    
    msg = ActuatorMotors()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    
    # 모터 출력 값 설정 (최대 12개 모터 지원, X500은 4개만 사용)
    # 값 범위: -1.0 ~ 1.0 (일반적으로 0.0 ~ 1.0 사용)
    motor_count = min(len(motor_outputs), 12)
    msg.control = [0.0] * 12  # 12개 모터 배열 초기화
    
    for i in range(motor_count):
        # 0.0 ~ 1.0 범위로 클램핑
        msg.control[i] = max(0.0, min(1.0, float(motor_outputs[i])))
    
    # 남은 모터들은 0.0으로 설정 (이미 초기화됨)
    
    node.actuator_motors_publisher.publish(msg)
    
    node.get_logger().debug(f"모터 출력 명령: {motor_outputs[:motor_count]}")


def calculate_adaptive_motor_outputs_for_fall(current_altitude, target_altitude, vertical_velocity, vertical_acceleration, hover_thrust=0.5):
    """
    실시간 속도/가속도 피드백을 사용한 적응형 자유낙하 제어
    
    Args:
        current_altitude: 현재 고도 (m)
        target_altitude: 목표 고도 (m)
        vertical_velocity: 현재 수직 속도 (m/s, NED 좌표계, 양수=하강)
        vertical_acceleration: 현재 수직 가속도 (m/s², NED 좌표계)
        hover_thrust: 호버링 기본 추력 (일반적으로 0.5)
        
    Returns:
        tuple: (motor_outputs, control_info)
            - motor_outputs: 4개 모터 출력값 리스트 [0.0-1.0]
            - control_info: 제어 상태 정보 딕셔너리
    """
    altitude_error = current_altitude - target_altitude
    
    # 제어 단계 구분
    if altitude_error > 10.0:
        # 단계 1: 자유낙하 단계 (10m 이상 거리)
        # 목표: 빠른 하강 속도 달성
        target_descent_rate = min(8.0, altitude_error * 0.5)  # 최대 8m/s 하강
        
        if vertical_velocity < target_descent_rate * 0.7:
            # 하강 속도가 부족하면 추력 더 감소
            thrust_factor = 0.0
        else:
            # 적절한 하강 속도면 약간의 추력 유지
            thrust_factor = 0.1
            
        control_mode = "FREEFALL"
        
    elif altitude_error > 3.0:
        # 단계 2: 감속 준비 단계 (3-10m 구간)
        # 목표: 점진적으로 하강 속도 감소
        distance_factor = (altitude_error - 3.0) / 7.0  # 1.0 → 0.0
        target_descent_rate = 2.0 + distance_factor * 4.0  # 6m/s → 2m/s
        
        # 속도 피드백 제어
        velocity_error = vertical_velocity - target_descent_rate
        
        if velocity_error > 1.0:
            # 너무 빠르게 하강 중 → 추력 증가
            thrust_factor = 0.2 + min(0.3, velocity_error * 0.1)
        elif velocity_error < -1.0:
            # 너무 느리게 하강 중 → 추력 감소
            thrust_factor = max(0.0, 0.2 + velocity_error * 0.1)
        else:
            # 적절한 속도 → 기본 추력
            thrust_factor = 0.2
            
        control_mode = "DECEL_PREP"
        
    else:
        # 단계 3: 최종 감속 단계 (3m 이내)
        # 목표: 안전한 착륙 속도로 감속
        target_descent_rate = max(0.5, altitude_error * 0.5)  # 1.5m/s → 0.5m/s
        
        # 더 강한 피드백 제어
        velocity_error = vertical_velocity - target_descent_rate
        
        # 기본 추력을 높게 시작
        base_thrust = 0.6
        velocity_correction = velocity_error * 0.2
        thrust_factor = max(0.3, min(0.9, base_thrust + velocity_correction))
        
        control_mode = "FINAL_DECEL"
    
    # 가속도 기반 미세 조정 (급격한 변화 방지)
    if abs(vertical_acceleration) > 15.0:  # 1.5G 이상의 급격한 가속도
        # 급격한 변화를 완화하기 위해 추력 조정
        if vertical_acceleration > 15.0:  # 급격한 감속
            thrust_factor *= 0.9  # 추력 약간 감소
        else:  # 급격한 가속
            thrust_factor *= 1.1  # 추력 약간 증가
            thrust_factor = min(1.0, thrust_factor)
    
    # 최종 모터 출력 계산
    final_thrust = hover_thrust * thrust_factor
    motor_outputs = [final_thrust, final_thrust, final_thrust, final_thrust]
    
    # 제어 정보 생성
    control_info = {
        'control_mode': control_mode,
        'altitude_error': altitude_error,
        'vertical_velocity': vertical_velocity,
        'vertical_acceleration': vertical_acceleration,
        'thrust_factor': thrust_factor,
        'target_descent_rate': target_descent_rate if 'target_descent_rate' in locals() else 0.0
    }
    
    return motor_outputs, control_info


def calculate_motor_outputs_for_freefall(hover_thrust=0.5, freefall_factor=0.0):
    """
    기존 단순한 자유낙하 모터 출력 계산 (하위 호환성 유지)
    
    Args:
        hover_thrust: 호버링을 위한 기본 추력 (일반적으로 0.5)
        freefall_factor: 자유낙하 정도 (0.0=완전 자유낙하, 1.0=정상 호버링)
        
    Returns:
        list: 4개 모터의 출력 값 [front_right, back_left, front_left, back_right]
    """
    # 자유낙하를 위해 모든 모터의 출력을 동일하게 감소
    output = hover_thrust * freefall_factor
    
    # X500 쿼드콥터의 4개 모터에 동일한 출력 적용
    return [output, output, output, output]


# ============================================
# 기동 데이터 기록 및 분석 함수들
# ============================================

class ManeuverDataLogger:
    """동적 기동 데이터 로거 클래스"""
    
    def __init__(self):
        self.data_log = []
        self.start_time = None
        
    def start_logging(self):
        """로깅 시작"""
        self.data_log = []
        self.start_time = time.time()
        
    def log_data_point(self, timestamp, altitude, velocity, acceleration, motor_outputs, control_info):
        """데이터 포인트 기록"""
        if self.start_time is None:
            self.start_time = timestamp
            
        data_point = {
            'time': timestamp - self.start_time,
            'altitude': altitude,
            'velocity_z': velocity,
            'acceleration_z': acceleration,
            'motor_outputs': motor_outputs.copy() if isinstance(motor_outputs, list) else motor_outputs,
            'control_info': control_info.copy() if isinstance(control_info, dict) else {}
        }
        
        self.data_log.append(data_point)
        
    def save_to_csv(self, filename):
        """CSV 파일로 저장"""
        import csv
        
        if not self.data_log:
            return False
            
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['time', 'altitude', 'velocity_z', 'acceleration_z', 
                         'motor_output_avg', 'control_mode', 'thrust_factor']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for data in self.data_log:
                motor_avg = sum(data['motor_outputs']) / len(data['motor_outputs'])
                control_info = data.get('control_info', {})
                
                writer.writerow({
                    'time': data['time'],
                    'altitude': data['altitude'],
                    'velocity_z': data['velocity_z'],
                    'acceleration_z': data['acceleration_z'],
                    'motor_output_avg': motor_avg,
                    'control_mode': control_info.get('control_mode', 'UNKNOWN'),
                    'thrust_factor': control_info.get('thrust_factor', 0.0)
                })
        
        return True
        
    def get_summary_stats(self):
        """요약 통계 계산"""
        if not self.data_log:
            return {}
            
        altitudes = [d['altitude'] for d in self.data_log]
        velocities = [d['velocity_z'] for d in self.data_log]
        accelerations = [d['acceleration_z'] for d in self.data_log]
        
        return {
            'total_time': self.data_log[-1]['time'],
            'altitude_drop': altitudes[0] - altitudes[-1],
            'max_descent_rate': max(velocities),
            'avg_descent_rate': sum(velocities) / len(velocities),
            'max_acceleration': max(accelerations),
            'data_points': len(self.data_log)
        }


def calculate_motor_outputs_for_pitch(hover_thrust=0.5, pitch_factor=0.0):
    """
    피치 기동(전진/후진)을 위한 모터 출력 값을 계산합니다.
    
    Args:
        hover_thrust: 호버링을 위한 기본 추력
        pitch_factor: 피치 강도 (-1.0=최대 후진, 0.0=중립, 1.0=최대 전진)
        
    Returns:
        list: 4개 모터의 출력 값 [front_right, back_left, front_left, back_right]
    """
    # 피치를 위해 앞/뒤 모터 출력을 다르게 설정
    front_thrust = hover_thrust - pitch_factor * 0.3  # 전진 시 앞 모터 감소
    back_thrust = hover_thrust + pitch_factor * 0.3   # 전진 시 뒤 모터 증가
    
    # 출력 범위 제한
    front_thrust = max(0.0, min(1.0, front_thrust))
    back_thrust = max(0.0, min(1.0, back_thrust))
    
    # [front_right, back_left, front_left, back_right]
    return [front_thrust, back_thrust, front_thrust, back_thrust]