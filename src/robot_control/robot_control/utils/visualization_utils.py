#!/usr/bin/env python3
"""
RViz 시각화 관련 유틸리티 함수 모음
마커 생성, 웨이포인트 시각화, 경로 표시 등의 기능을 제공합니다.
"""

import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3


def create_marker(node, frame_id, ns, marker_id, marker_type, pose, scale, color, text="", lifetime_sec=1.1):
    """
    RViz 시각화를 위한 기본 마커를 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        frame_id: 좌표계 프레임 ID
        ns: 마커 네임스페이스
        marker_id: 마커 ID
        marker_type: 마커 타입 (Marker.SPHERE, Marker.CYLINDER 등)
        pose: 마커 위치 (Pose 객체)
        scale: 마커 크기 (Vector3 객체)
        color: 마커 색상 (ColorRGBA 객체)
        text: 텍스트 마커인 경우 표시할 텍스트
        lifetime_sec: 마커 수명 (초)
        
    Returns:
        Marker: 생성된 마커 객체
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale = scale
    marker.color = color
    
    if text:
        marker.text = text
    
    if lifetime_sec > 0:
        marker.lifetime = rclpy.duration.Duration(seconds=lifetime_sec).to_msg()
    
    return marker


def create_target_visual(node, target_id, position, color, text_label="", ns_prefix="target"):
    """
    하나의 타겟을 나타내는 시각적 요소(구, 텍스트)들을 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        target_id: 타겟 ID
        position: 위치 [x, y, z]
        color: 색상 튜플 (r, g, b)
        text_label: 텍스트 라벨
        ns_prefix: 네임스페이스 접두사
        
    Returns:
        list: 마커 리스트 [sphere, text]
    """
    markers = []
    
    # 1. 구 마커 (반투명)
    sphere_pose = Pose()
    sphere_pose.position = Point(x=position[0], y=position[1], z=position[2])
    sphere_pose.orientation.w = 1.0
    
    sphere_scale = Vector3(x=1.0, y=1.0, z=0.5)  # 지름 2m 구
    sphere_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.25)
    
    sphere_marker = create_marker(
        node, "map", f"{ns_prefix}_spheres", target_id, 
        Marker.SPHERE, sphere_pose, sphere_scale, sphere_color
    )
    markers.append(sphere_marker)
    
    # 2. 텍스트 마커
    if text_label:
        text_pose = Pose()
        text_pose.position = Point(x=position[0], y=position[1], z=position[2] - 1.0)  # 1m 아래
        text_pose.orientation.w = 1.0
        
        text_scale = Vector3(x=0.0, y=0.0, z=0.3)  # 텍스트 크기
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 흰색
        
        text_marker = create_marker(
            node, "map", f"{ns_prefix}_texts", target_id,
            Marker.TEXT_VIEW_FACING, text_pose, text_scale, text_color, text_label
        )
        markers.append(text_marker)
    
    return markers


def create_waypoint_visual(node, waypoint_id, position, waypoint_status="future", text_label="", ns_prefix="waypoint"):
    """
    하나의 웨이포인트를 나타내는 단순한 큐브 마커를 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        waypoint_id: 웨이포인트 ID
        position: 위치 [x, y, z]
        waypoint_status: 웨이포인트 상태 ("future": 초록, "current": 노랑, "passed": 반투명 회색)
        text_label: 텍스트 라벨
        ns_prefix: 네임스페이스 접두사
        
    Returns:
        list: 마커 리스트 [cube, text]
    """
    markers = []
    
    # 상태에 따른 색상 설정
    if waypoint_status == "current":
        color = (1.0, 1.0, 0.0)  # 노란색
        alpha = 0.9
    elif waypoint_status == "passed":
        color = (0.5, 0.5, 0.5)  # 회색
        alpha = 0.3  # 반투명
    else:  # "future"
        color = (0.0, 1.0, 0.0)  # 초록색
        alpha = 0.5
    
    # 큐브 마커
    cube_pose = Pose()
    cube_pose.position = Point(x=position[0], y=position[1], z=position[2])
    cube_pose.orientation.w = 1.0
    
    cube_scale = Vector3(x=0.5, y=0.5, z=0.5)  # 1.5m 큐브
    cube_color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    
    cube_marker = create_marker(
        node, "map", f"{ns_prefix}_cubes", waypoint_id, 
        Marker.CUBE, cube_pose, cube_scale, cube_color
    )
    markers.append(cube_marker)
    
    # 텍스트 마커
    if text_label:
        text_pose = Pose()
        text_pose.position = Point(x=position[0], y=position[1], z=position[2] + 1.0)  # 1m 위
        text_pose.orientation.w = 1.0
        
        text_scale = Vector3(x=0.0, y=0.0, z=0.3)  # 텍스트 크기
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 흰색
        
        text_marker = create_marker(
            node, "map", f"{ns_prefix}_texts", waypoint_id,
            Marker.TEXT_VIEW_FACING, text_pose, text_scale, text_color, text_label
        )
        markers.append(text_marker)
    
    return markers


def create_path_marker(node, waypoints, color=(0.1, 1.0, 0.1), ns="path", marker_id=0, alpha=1.0):
    """
    웨이포인트들을 연결하는 경로 마커를 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        waypoints: 웨이포인트 리스트 [[x, y, z], ...]
        color: 경로 색상 (r, g, b)
        ns: 네임스페이스
        marker_id: 마커 ID
        alpha: 경로 투명도
        
    Returns:
        Marker: 경로 마커
    """
    path_marker = Marker()
    path_marker.header.frame_id = "map"
    path_marker.header.stamp = node.get_clock().now().to_msg()
    path_marker.ns = ns
    path_marker.id = marker_id
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.2  # 선 두께
    path_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    path_marker.lifetime = rclpy.duration.Duration(seconds=1.1).to_msg()
    
    # 웨이포인트들을 연결하는 점들 추가
    for wp in waypoints:
        point = Point(x=wp[0], y=wp[1], z=wp[2])
        path_marker.points.append(point)
    
    return path_marker


def create_mission_visual_markers(node, waypoints, stare_targets=None, current_waypoint_idx=0):
    """
    미션 전체의 시각화 마커들을 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        waypoints: 드론 웨이포인트 리스트 [[x, y, z], ...]
        stare_targets: 응시 타겟 리스트 [[x, y, z], ...] (선택사항)
        current_waypoint_idx: 현재 웨이포인트 인덱스
        
    Returns:
        MarkerArray: 모든 시각화 마커들
    """
    marker_array = MarkerArray()
    
    # 1. 웨이포인트 경로 마커
    if len(waypoints) > 1:
        # 전체 경로는 반투명하게 표시
        path_marker = create_path_marker(node, waypoints, ns="waypoint_path_transparent", marker_id=0, alpha=0.3)
        marker_array.markers.append(path_marker)
        
        # 현재 진행중인 경로는 불투명하게 표시
        if current_waypoint_idx > 0 and current_waypoint_idx < len(waypoints):
            current_path_segment = [waypoints[current_waypoint_idx - 1], waypoints[current_waypoint_idx]]
            current_path_marker = create_path_marker(node, current_path_segment, ns="waypoint_path_current", marker_id=1, alpha=1.0)
            marker_array.markers.append(current_path_marker)
    
    # 2. 웨이포인트 마커들
    for i, wp in enumerate(waypoints):
        # 웨이포인트 상태 결정
        if i < current_waypoint_idx:
            waypoint_status = "passed"
        elif i == current_waypoint_idx:
            waypoint_status = "current"
        else:
            waypoint_status = "future"
        
        text_label = f"WP{i}\n({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})"
        wp_markers = create_waypoint_visual(node, i, wp, waypoint_status, text_label, "waypoints")
        marker_array.markers.extend(wp_markers)
    
    # 3. Stare 타겟 마커들 (있는 경우)
    if stare_targets:
        stare_target_color = (1.0, 1.0, 1.0)  # 흰색
        for i, target in enumerate(stare_targets):
            text_label = f"Target {i}\n({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})"
            target_markers = create_target_visual(node, i, target, stare_target_color, text_label, "stare_targets")
            marker_array.markers.extend(target_markers)
    
    return marker_array


def create_interactive_mission_markers(node, drone_waypoints, stare_targets, final_destination=None):
    """
    대화형 미션용 드론 웨이포인트와 주시 타겟 마커들을 생성합니다.
    
    Args:
        node: ROS2 노드 인스턴스
        drone_waypoints: 드론 웨이포인트 리스트 [[x, y, z], ...]
        stare_targets: 주시 타겟 리스트 [[x, y, z], ...]
        final_destination: 최종 목적지 [x, y, z] (선택사항)
        
    Returns:
        MarkerArray: 모든 마커들을 포함한 배열
    """
    marker_array = MarkerArray()
    
    # 1. 드론 웨이포인트들 (녹색 구체들)
    for i, waypoint in enumerate(drone_waypoints):
        text_label = f"DroneWP{i}\n({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f})"
        
        # 드론 웨이포인트는 녹색으로 표시
        markers = create_waypoint_visual(node, i, waypoint, "future", text_label, "drone_waypoints")
        marker_array.markers.extend(markers)
    
    # 2. 주시 타겟들 (다양한 색상의 타겟 형태)
    stare_target_color = (1.0, 1.0, 1.0)  # 흰색
    for i, target in enumerate(stare_targets):
        text_label = f"Stare{i}\n({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})"
        
        target_markers = create_target_visual(node, i, target, stare_target_color, text_label, "stare_targets")
        marker_array.markers.extend(target_markers)
    
    # 3. 최종 목적지 강조 표시 (있는 경우)
    if final_destination:
        final_text = f"FINAL\n({final_destination[0]:.1f}, {final_destination[1]:.1f}, {final_destination[2]:.1f})"
        
        # 최종 목적지는 특별한 색상(골드)으로 표시
        final_markers = create_target_visual(node, 999, final_destination, (1.0, 0.8, 0.0), final_text, "final_destination")
        marker_array.markers.extend(final_markers)
    
    return marker_array 