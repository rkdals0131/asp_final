#!/usr/bin/env python3
"""
Pure Pursuit 경로 추종 모듈
Pure Pursuit 알고리즘을 사용한 경로 추종 제어를 담당
"""

import math
from typing import List, Tuple, Optional
from geometry_msgs.msg import Twist


class PurePursuitController:
    """
    Pure Pursuit 알고리즘 기반 경로 추종 제어기
    전방주시거리를 사용하여 목표점을 찾고 조향각을 계산
    """
    
    def __init__(self, lookahead_k: float = 2.0, lookahead_min: float = 0.5, 
                 lookahead_max: float = 3.0):
        """
        Args:
            lookahead_k: 전방주시거리 계수 (lookahead = k * velocity)
            lookahead_min: 최소 전방주시거리 (m)
            lookahead_max: 최대 전방주시거리 (m)
        """
        self.lookahead_k = lookahead_k
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
    
    def calculate_control_command(self, current_pos: Tuple[float, float], 
                                 current_yaw: float, current_speed: float,
                                 path_points: List[Tuple[float, float]], 
                                 target_velocities: List[float], 
                                 last_closest_idx_ref: List[int]) -> Tuple[Twist, Optional[Tuple[float, float]]]:
        """
        Pure Pursuit 제어 명령 계산 (원본 로직과 동일)
        
        Args:
            current_pos: 현재 위치 (x, y)
            current_yaw: 현재 방향각 (rad)
            current_speed: 현재 속도 (m/s)
            path_points: 경로점 목록 [(x, y), ...]
            target_velocities: 목표 속도 목록
            
        Returns:
            (Twist 명령, 목표점) 튜플
        """
        if not path_points or not target_velocities:
            return Twist(), None
        
        # 원본: goal_idx, lookahead_dist = self._find_goal_point(current_x, current_y)
        closest_idx = self.find_closest_point_idx(current_pos, path_points, last_closest_idx_ref)
        lookahead_dist = self._calculate_lookahead_distance(current_speed)
        goal_idx = self.find_goal_point(current_pos, path_points, closest_idx, lookahead_dist)
        
        if goal_idx is None or goal_idx >= len(target_velocities):
            return Twist(), None
        
        # 원본 로직과 동일
        goal_x, goal_y = path_points[goal_idx]
        target_speed = target_velocities[goal_idx]
        alpha = math.atan2(goal_y - current_pos[1], goal_x - current_pos[0]) - current_yaw
        effective_speed = max(current_speed, 0.1)
        angular_z = (2.0 * effective_speed * math.sin(alpha)) / lookahead_dist
        
        # Twist 명령 생성
        cmd_vel = Twist()
        cmd_vel.linear.x = float(target_speed)
        cmd_vel.angular.z = float(angular_z)
        
        return cmd_vel, (goal_x, goal_y)
    
    def find_closest_point_idx(self, current_pos: Tuple[float, float], 
                              path_points: List[Tuple[float, float]], 
                              last_closest_idx_ref: List[int]) -> int:
        """
        현재 위치에서 가장 가까운 경로점의 인덱스 찾기 (원본 로직과 동일)
        
        Args:
            current_pos: 현재 위치 (x, y)
            path_points: 경로점 목록
            
        Returns:
            가장 가까운 점의 인덱스
        """
        if not path_points:
            return 0
        
        # 원본: search_end = min(self.last_closest_idx + 200, len(self.full_path_points))
        last_closest_idx = last_closest_idx_ref[0]
        search_end = min(last_closest_idx + 200, len(path_points))
        path_segment = path_points[last_closest_idx:search_end]
        
        if len(path_segment) == 0:
            return last_closest_idx
        
        # numpy를 사용한 벡터화 계산 (원본과 동일)
        import numpy as np
        path_array = np.array(path_segment)
        current_array = np.array([current_pos[0], current_pos[1]])
        dists = np.linalg.norm(path_array - current_array, axis=1)
        
        closest_idx = last_closest_idx + int(np.argmin(dists))
        last_closest_idx_ref[0] = closest_idx  # 참조 업데이트
        return closest_idx
    
    def find_goal_point(self, current_pos: Tuple[float, float], 
                       path_points: List[Tuple[float, float]], 
                       start_idx: int, lookahead_dist: float) -> Optional[int]:
        """
        전방주시거리에 해당하는 목표점 인덱스 찾기 (원본 로직과 동일)
        
        Args:
            current_pos: 현재 위치 (x, y)
            path_points: 경로점 목록
            start_idx: 검색 시작 인덱스 (closest_idx)
            lookahead_dist: 전방주시거리
            
        Returns:
            목표점 인덱스 또는 None
        """
        if not path_points:
            return None
        
        # 원본: for i in range(closest_idx, len(self.full_path_points)):
        for i in range(start_idx, len(path_points)):
            dist = math.hypot(
                current_pos[0] - path_points[i][0],
                current_pos[1] - path_points[i][1]
            )
            
            if dist >= lookahead_dist:
                return i
        
        # 전방주시거리 내에 점이 없으면 마지막 인덱스 반환
        return len(path_points) - 1
    
    def _calculate_lookahead_distance(self, current_speed: float) -> float:
        """
        현재 속도에 기반한 전방주시거리 계산 (원본 로직과 동일)
        
        Args:
            current_speed: 현재 속도 (m/s)
            
        Returns:
            전방주시거리 (m)
        """
        # 원본: lookahead_dist = np.clip(self.LOOKAHEAD_K * self.current_speed + self.LOOKAHEAD_MIN, self.LOOKAHEAD_MIN, self.LOOKAHEAD_MAX)
        lookahead = self.lookahead_k * current_speed + self.lookahead_min
        return max(self.lookahead_min, min(self.lookahead_max, lookahead))
    

    
    def is_path_complete(self, current_pos: Tuple[float, float], 
                        path_points: List[Tuple[float, float]], 
                        completion_threshold: float = 1.5) -> bool:
        """
        경로 완주 여부 확인
        
        Args:
            current_pos: 현재 위치 (x, y)
            path_points: 경로점 목록
            completion_threshold: 완주 판정 임계거리 (m)
            
        Returns:
            경로 완주 여부
        """
        if not path_points:
            return True
        
        # 마지막 점까지의 거리 확인
        final_point = path_points[-1]
        dist_to_final = math.hypot(
            current_pos[0] - final_point[0],
            current_pos[1] - final_point[1]
        )
        
        return dist_to_final <= completion_threshold
    
 