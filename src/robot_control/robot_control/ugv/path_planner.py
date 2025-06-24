#!/usr/bin/env python3
"""
경로 계획 모듈
웨이포인트에서 부드러운 경로를 생성하는 로직을 담당
"""

import math
import numpy as np
from scipy.interpolate import splprep, splev
from typing import List, Tuple


class PathPlanner:
    """
    웨이포인트 목록에서 부드러운 경로를 생성하는 클래스
    Spline 보간을 사용하여 연속적인 경로점들을 생성
    """
    
    def __init__(self, path_density: float = 0.1):
        """
        Args:
            path_density: 경로점 간 거리 (미터)
        """
        self.path_density = path_density
    
    def generate_path_from_waypoints(self, waypoints: List[Tuple]) -> List[Tuple[float, float]]:
        """
        웨이포인트 목록에서 부드러운 경로 생성
        
        Args:
            waypoints: 웨이포인트 목록 [(x, y, ...), ...]
            
        Returns:
            경로점 목록 [(x, y), ...]
        """
        if len(waypoints) < 2:
            return []
        
        # 웨이포인트에서 x, y 좌표만 추출
        wx = [float(wp[0]) for wp in waypoints]
        wy = [float(wp[1]) for wp in waypoints]
        
        if len(waypoints) == 2:
            # 두 점만 있는 경우 직선 경로 생성
            return self._generate_straight_path((wx[0], wy[0]), (wx[1], wy[1]))
        
        try:
            # Spline 보간으로 부드러운 경로 생성
            tck, _ = splprep([wx, wy], s=0.5, k=min(3, len(waypoints)-1))
        except Exception:
            # Spline 실패 시 직선 경로들의 연결로 대체
            path_points = []
            for i in range(len(waypoints) - 1):
                segment = self._generate_straight_path((wx[i], wy[i]), (wx[i+1], wy[i+1]))
                if i > 0 and segment:
                    segment = segment[1:]  # 중복점 제거
                path_points.extend(segment)
            return path_points
        
        # 경로 길이 기반으로 적절한 점 개수 계산
        path_len = np.sum(np.sqrt(np.diff(wx)**2 + np.diff(wy)**2))
        num_points = max(2, int(path_len / self.path_density))
        
        # Spline 보간으로 경로점 생성
        u_fine = np.linspace(0, 1, num_points)
        x_fine, y_fine = splev(u_fine, tck)
        
        return list(zip(x_fine, y_fine))
    
    def _generate_straight_path(self, start_pos: Tuple[float, float], 
                               end_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        두 점 사이의 직선 경로 생성
        
        Args:
            start_pos: 시작점 (x, y)
            end_pos: 끝점 (x, y)
            
        Returns:
            직선 경로점 목록 [(x, y), ...]
        """
        dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
        
        if dist < self.path_density:
            return []
        
        num_points = max(2, int(dist / self.path_density))
        x_points = np.linspace(start_pos[0], end_pos[0], num_points)
        y_points = np.linspace(start_pos[1], end_pos[1], num_points)
        
        return list(zip(x_points, y_points))
    
    def calculate_path_distances(self, path_points: List[Tuple[float, float]]) -> List[float]:
        """
        경로점들 사이의 누적 거리 계산
        
        Args:
            path_points: 경로점 목록 [(x, y), ...]
            
        Returns:
            누적 거리 목록 [0.0, d1, d1+d2, ...]
        """
        if not path_points:
            return []
        
        distances = [0.0]
        for i in range(1, len(path_points)):
            dist = math.hypot(
                path_points[i][0] - path_points[i-1][0],
                path_points[i][1] - path_points[i-1][1]
            )
            distances.append(distances[-1] + dist)
        
        return distances
    
    def calculate_curvature(self, path_points: List[Tuple[float, float]]) -> List[float]:
        """
        경로의 각 점에서 곡률 계산
        
        Args:
            path_points: 경로점 목록 [(x, y), ...]
            
        Returns:
            곡률 목록
        """
        if len(path_points) < 3:
            return [0.0] * len(path_points)
        
        x = np.array([p[0] for p in path_points])
        y = np.array([p[1] for p in path_points])
        
        # 1차 및 2차 미분 계산
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # 곡률 계산: κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        curvature = (dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
        
        return curvature.tolist() 