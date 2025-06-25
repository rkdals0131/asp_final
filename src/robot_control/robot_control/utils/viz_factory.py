#!/usr/bin/env python3
"""
RViz 시각화에 사용될 Marker 및 MarkerArray 객체 생성을 전담하는 팩토리 모음.
각 함수는 ROS 노드의 상태에 의존하지 않으며, 필요한 모든 데이터는 인자를 통해 전달받는다.
이를 통해 시각화 '로직'과 '데이터'를 분리하고, 코드의 재사용성을 높인다.
"""

import math
import csv
import os
from typing import Tuple
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Path

# ==============================================================================
# 기본 마커 생성 유틸리티 (기존 visualization_utils.py의 저수준 함수)
# ==============================================================================

def create_base_marker(header: Header, ns: str, marker_id: int, marker_type: int,
                       pose: Pose, scale: Vector3, color: ColorRGBA, text: str = "", lifetime_sec: float = 0.0) -> Marker:
    """
    모든 마커의 기본이 되는 가장 기본적인 마커를 생성.
    
    Args:
        header: 마커에 사용할 ROS 메시지 헤더 (frame_id, stamp 포함)
        ns: 마커 네임스페이스
        marker_id: 마커 ID
        marker_type: 마커 타입 (e.g., Marker.SPHERE)
        pose: 마커의 Pose
        scale: 마커의 크기 (Vector3)
        color: 마커의 색상 (ColorRGBA)
        text: 텍스트 마커일 경우 표시할 텍스트
        lifetime_sec: 마커의 수명 (0은 영구)
        
    Returns:
        Marker: 생성된 마커 객체
    """
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale = scale
    marker.color = color
    
    if text:
        marker.text = text
    
    # rclpy 사용을 피하기 위해 직접 Duration 메시지 생성 회피 (노드에서 설정)
    # if lifetime_sec > 0:
    #     marker.lifetime = rclpy.duration.Duration(seconds=lifetime_sec).to_msg()
    
    return marker

def euler_to_quaternion(roll, pitch, yaw):
    """오일러 각도를 쿼터니언으로 변환 (Extrinsic ZYX)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy - sr * sp * sy
    qx = sr * cp * cy + cr * sp * sy
    qy = cr * sp * cy - sr * cp * sy
    qz = cr * cp * sy + sr * sp * cy
    return qx, qy, qz, qw

# ==============================================================================
# PathFollowerNode 관련 마커 생성 (from path_follower_node.py)
# ==============================================================================

def create_ugv_path_marker(header: Header, full_path_points: list) -> Path:
    """UGV의 전체 경로를 나타내는 Path 메시지를 생성."""
    path_msg = Path()
    path_msg.header = header
    for point in full_path_points:
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose = Pose()
        pose.position.x, pose.position.y, pose.orientation.w = point[0], point[1], 1.0
        pose_stamped.pose = pose
        path_msg.poses.append(pose_stamped)
    return path_msg

def create_ugv_waypoint_markers(header: Header, raw_waypoints: list) -> MarkerArray:
    """UGV의 원본 웨이포인트를 시각화하는 마커 배열을 생성."""
    marker_array = MarkerArray()
    for i, waypoint in enumerate(raw_waypoints):
        x, y = waypoint[0], waypoint[1]
        
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), 0.5
        pose.orientation.w = 1.0
        
        scale = Vector3(x=1.5, y=1.5, z=1.5)
        
        color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8) # 기본 파란색
        if len(waypoint) > 2:
            if waypoint[2] == 2: color.r, color.g = 1.0, 1.0 # 노란색 (드론 이륙)
            elif waypoint[2] == 4: color.r, color.g, color.b = 1.0, 0.0, 0.0 # 빨간색 (종료)
        if len(waypoint) > 3 and 0 < waypoint[3] <= 1.0:
            color.r, color.g = 1.0, 0.5 # 주황색 (감속)

        marker = create_base_marker(header, "ugv_waypoints", i, Marker.SPHERE, pose, scale, color)
        marker_array.markers.append(marker)
        
    return marker_array

def create_speed_markers(header: Header, full_path_points: list, full_target_velocities: list, path_density: float, max_speed: float) -> MarkerArray:
    """경로 각 지점의 목표 속도를 시각화하는 마커(실린더, 텍스트) 배열을 생성."""
    marker_array = MarkerArray()
    step = max(1, int(0.5 / path_density))

    # Clear previous markers
    delete_marker = Marker(action=Marker.DELETEALL)
    marker_array.markers.append(delete_marker)

    for i in range(0, len(full_path_points), step):
        if i >= len(full_target_velocities): break
        point, vel = full_path_points[i], full_target_velocities[i]
        
        # Speed Cylinder
        cyl_height = max(0.2, float(vel * 0.5))
        cyl_pose = Pose()
        cyl_pose.position.x, cyl_pose.position.y, cyl_pose.position.z = point[0], point[1], cyl_height / 2.0
        cyl_pose.orientation.w = 1.0
        cyl_scale = Vector3(x=0.15, y=0.15, z=cyl_height)
        
        cyl_color = ColorRGBA(a=0.6)
        if vel <= 1.0: cyl_color.r, cyl_color.g, cyl_color.b = 0.0, 0.5, 1.0
        elif vel <= max_speed * 0.6: cyl_color.r, cyl_color.g, cyl_color.b = 0.0, 1.0, 0.0
        else: cyl_color.r, cyl_color.g, cyl_color.b = 1.0, 0.0, 0.0
        
        cylinder = create_base_marker(header, "speed_cylinders", i, Marker.CYLINDER, cyl_pose, cyl_scale, cyl_color)
        marker_array.markers.append(cylinder)

        # Speed Text
        text_pose = Pose()
        text_pose.position.x, text_pose.position.y, text_pose.position.z = point[0], point[1], cyl_height + 0.3
        text_pose.orientation.w = 1.0
        text_scale = Vector3(z=0.3)
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        text = create_base_marker(header, "speed_text", i, Marker.TEXT_VIEW_FACING, text_pose, text_scale, text_color, f"{vel:.1f}")
        marker_array.markers.append(text)
        
    return marker_array

def create_lookahead_marker(header: Header, goal_point: tuple) -> Marker:
    """Pure Pursuit의 Lookahead 지점을 시각화하는 마커를 생성."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = goal_point[0], goal_point[1], 0.5
    pose.orientation.w = 1.0
    scale = Vector3(x=1.0, y=1.0, z=1.0)
    color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) # 초록색
    
    marker = create_base_marker(header, "lookahead_point", 0, Marker.SPHERE, pose, scale, color)
    return marker

# ==============================================================================
# ArUco 마커 시각화 관련 (from marker_visual.py)
# ==============================================================================

def create_ground_truth_markers_from_csv(header: Header, csv_path: str) -> Tuple[MarkerArray, str]:
    """
    CSV 파일 경로를 직접 받아 Ground-Truth 마커 데이터를 로드, 파싱하고 MarkerArray를 생성.
    
    Args:
        header: 마커에 사용할 ROS 메시지 헤더
        csv_path: Ground-Truth 데이터가 담긴 CSV 파일의 절대 경로

    Returns:
        Tuple[MarkerArray, str]: 생성된 마커 배열과 로깅을 위한 정보 문자열을 반환.
                                 실패 시 빈 MarkerArray와 에러 메시지를 반환.
    """
    if not os.path.exists(csv_path):
        error_msg = f"Ground Truth CSV file not found: {csv_path}"
        return MarkerArray(), error_msg

    ground_truth_data = {}
    try:
        with open(csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                marker_name = row['name']
                if marker_name.startswith('aruco_marker_'):
                    marker_id_str = marker_name.replace('aruco_marker_', '').replace('_', '')
                    marker_id = int(marker_id_str)
                    roll, pitch, yaw = float(row['roll']), float(row['pitch']), float(row['yaw'])
                    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                    ground_truth_data[marker_id] = {
                        'x': float(row['e']), 'y': float(row['n']), 'z': float(row['u']),
                        'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                    }
        info_msg = f"Loaded {len(ground_truth_data)} ground truth markers from {csv_path}"
        
        marker_array = create_ground_truth_markers(header, ground_truth_data)
        return marker_array, info_msg

    except Exception as e:
        error_msg = f"Failed to load/parse ground truth CSV: {e}"
        return MarkerArray(), error_msg

def create_ground_truth_markers(header: Header, ground_truth_data: dict) -> MarkerArray:
    """CSV에서 로드한 Ground-Truth 마커 정보를 시각화."""
    marker_array = MarkerArray()
    for marker_id, position in ground_truth_data.items():
        pose = Pose()
        pose.position.x = position['x']
        pose.position.y = position['y']
        pose.position.z = position['z']
        pose.orientation.x = position['qx']
        pose.orientation.y = position['qy']
        pose.orientation.z = position['qz']
        pose.orientation.w = position['qw']
        
        scale = Vector3(x=3.0, y=3.0, z=3.0) # 반경 1.5m
        color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.3) # 청록색, 반투명
        
        marker = create_base_marker(header, "ground_truth_markers", marker_id, Marker.SPHERE, pose, scale, color)
        marker_array.markers.append(marker)
        
    return marker_array

def create_detected_markers(header: Header, stored_markers: dict, current_marker_ids: set) -> MarkerArray:
    """탐지된 ArUco 마커를 시각화."""
    marker_array = MarkerArray()
    for marker_id_str, pose in stored_markers.items():
        marker_id = int(marker_id_str)
        
        scale = Vector3(x=0.5, y=0.5, z=0.5)
        
        color = ColorRGBA()
        if marker_id_str in current_marker_ids:
            # 현재 프레임에 탐지된 마커: 불투명 핑크색
            color.r, color.g, color.b, color.a = 1.0, 0.0, 1.0, 1.0
        else:
            # 이전 프레임에서 탐지된 마커: 어두운 노란색
            color.r, color.g, color.b, color.a = 0.8, 0.8, 0.0, 0.8

        # ID 충돌 방지를 위해 100을 더함
        marker = create_base_marker(header, "detected_markers", marker_id + 100, Marker.SPHERE, pose, scale, color)
        marker_array.markers.append(marker)
        
    return marker_array

# ==============================================================================
# 미션 시각화 관련 (from visualization_utils.py)
# ==============================================================================

def create_waypoint_visual(header: Header, waypoint_id: int, position: list,
                           waypoint_status: str="future", text_label: str="", ns_prefix: str="waypoint"):
    """하나의 웨이포인트(큐브 + 텍스트)를 나타내는 마커 리스트를 생성."""
    markers = []
    
    # 상태에 따른 색상 설정
    if waypoint_status == "current": color, alpha = (1.0, 1.0, 0.0), 0.9
    elif waypoint_status == "passed": color, alpha = (0.5, 0.5, 0.5), 0.3
    else: color, alpha = (0.0, 1.0, 0.0), 0.5
    
    # 큐브 마커
    cube_pose = Pose()
    cube_pose.position = Point(x=position[0], y=position[1], z=position[2])
    cube_pose.orientation.w = 1.0
    cube_scale = Vector3(x=0.5, y=0.5, z=0.5)
    cube_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    
    cube_marker = create_base_marker(header, f"{ns_prefix}_cubes", waypoint_id, Marker.CUBE, cube_pose, cube_scale, cube_color)
    markers.append(cube_marker)
    
    # 텍스트 마커
    if text_label:
        text_pose = Pose()
        text_pose.position = Point(x=position[0], y=position[1], z=position[2] + 1.0)
        text_pose.orientation.w = 1.0
        text_scale = Vector3(z=0.3)
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        text_marker = create_base_marker(header, f"{ns_prefix}_texts", waypoint_id, Marker.TEXT_VIEW_FACING, text_pose, text_scale, text_color, text_label)
        markers.append(text_marker)
        
    return markers

def create_mission_path_marker(header: Header, waypoints: list, color=(0.1, 1.0, 0.1), ns="path", marker_id=0, alpha=1.0):
    """웨이포인트들을 연결하는 경로(LINE_STRIP) 마커를 생성."""
    path_marker = Marker()
    path_marker.header = header
    path_marker.ns = ns
    path_marker.id = marker_id
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.2
    path_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    
    for wp in waypoints:
        path_marker.points.append(Point(x=wp[0], y=wp[1], z=wp[2]))
        
    return path_marker

def create_target_visual(header, target_id, position, color, text_label="", ns_prefix="target"):
    """하나의 타겟(구 + 텍스트)을 나타내는 마커 리스트를 생성."""
    markers = []
    
    # 구 마커 (반투명)
    sphere_pose = Pose()
    sphere_pose.position = Point(x=position[0], y=position[1], z=position[2])
    sphere_pose.orientation.w = 1.0
    sphere_scale = Vector3(x=1.0, y=1.0, z=0.5)
    sphere_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.25)
    
    sphere_marker = create_base_marker(header, f"{ns_prefix}_spheres", target_id, Marker.SPHERE, sphere_pose, sphere_scale, sphere_color)
    markers.append(sphere_marker)
    
    # 텍스트 마커
    if text_label:
        text_pose = Pose()
        text_pose.position = Point(x=position[0], y=position[1], z=position[2] - 1.0)
        text_pose.orientation.w = 1.0
        text_scale = Vector3(z=0.3)
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        text_marker = create_base_marker(header, f"{ns_prefix}_texts", target_id, Marker.TEXT_VIEW_FACING, text_pose, text_scale, text_color, text_label)
        markers.append(text_marker)
        
    return markers 