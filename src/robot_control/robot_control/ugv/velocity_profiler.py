#!/usr/bin/env python3
"""
속도 프로파일 모듈
S-Curve 속도 프로파일 생성 및 곡률 기반 속도 제한을 담당
"""

import math
import numpy as np
from typing import List, Tuple


class VelocityProfiler:
    """
    경로에 대한 S-Curve 속도 프로파일을 생성하는 클래스
    곡률, 가속도, 저크 제한을 고려하여 최적화된 속도 프로파일 생성
    """
    
    def __init__(self, max_speed: float = 6.0, min_speed: float = 0.5,
                 max_accel: float = 2.0, max_decel: float = 1.0, max_jerk: float = 3.0,
                 max_lateral_accel: float = 2.0):
        """
        Args:
            max_speed: 최대 속도 (m/s)
            min_speed: 최소 속도 (m/s)
            max_accel: 최대 가속도 (m/s²)
            max_decel: 최대 감속도 (m/s²)
            max_jerk: 최대 저크 (m/s³)
            max_lateral_accel: 최대 횡방향 가속도 (m/s²)
        """
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.max_lateral_accel = max_lateral_accel
    
    def update_dynamic_limits(self, max_accel: float, max_jerk: float):
        """
        동적으로 가속도 및 저크 제한 업데이트 (드론 탑재 여부에 따라)
        
        Args:
            max_accel: 새로운 최대 가속도
            max_jerk: 새로운 최대 저크
        """
        self.max_accel = max_accel
        self.max_jerk = max_jerk
    
    def generate_scurve_profile(self, path_points: List[Tuple[float, float]], 
                               distances: List[float], curvatures: List[float],
                               waypoints: List[Tuple], start_vel: float = 0.0, 
                               end_vel: float = 0.0) -> List[float]:
        """
        S-Curve 속도 프로파일 생성
        
        Args:
            path_points: 경로점 목록 [(x, y), ...]
            distances: 누적 거리 목록
            curvatures: 곡률 목록
            waypoints: 원본 웨이포인트 정보 (속도 제한용)
            start_vel: 시작 속도
            end_vel: 종료 속도
            
        Returns:
            속도 프로파일 목록
        """
        if not path_points or len(path_points) < 2:
            return []
        
        num_points = len(path_points)
        
        # 1단계: 곡률 기반 속도 제한 계산
        velocity_limits = self._calculate_curvature_limited_velocities(curvatures)
        
        # 2단계: 웨이포인트 기반 속도 제한 적용
        velocity_limits = self._apply_waypoint_speed_limits(
            velocity_limits, path_points, waypoints
        )
        
        # 3단계: 사다리꼴 속도 프로파일 생성 (가속도 제한)
        trapezoidal_profile = self._generate_trapezoidal_profile(
            velocity_limits, distances, start_vel, end_vel
        )
        
        # 4단계: S-Curve 적용 (저크 제한)
        scurve_profile = self._apply_scurve_smoothing(
            trapezoidal_profile, distances
        )
        
        return scurve_profile
    
    def _calculate_curvature_limited_velocities(self, curvatures: List[float]) -> List[float]:
        """
        곡률 기반 속도 제한 계산
        
        Args:
            curvatures: 곡률 목록
            
        Returns:
            곡률 제한 속도 목록
        """
        velocities = []
        
        for k in curvatures:
            abs_k = abs(k)
            if abs_k < 1e-6:
                # 직선 구간
                max_v = self.max_speed
            else:
                # 곡선 구간: v = sqrt(a_lateral / |κ|)
                max_v = math.sqrt(self.max_lateral_accel / abs_k)
            
            velocities.append(np.clip(max_v, 0.0, self.max_speed))
        
        return velocities
    
    def _apply_waypoint_speed_limits(self, velocity_limits: List[float], 
                                   path_points: List[Tuple[float, float]], 
                                   waypoints: List[Tuple]) -> List[float]:
        """
        웨이포인트 기반 속도 제한 적용
        
        Args:
            velocity_limits: 기존 속도 제한 목록
            path_points: 경로점 목록
            waypoints: 웨이포인트 목록
            
        Returns:
            업데이트된 속도 제한 목록
        """
        updated_limits = list(velocity_limits)
        num_points = len(path_points)
        
        for waypoint in waypoints:
            wp_x, wp_y = waypoint[0], waypoint[1]
            
            # 가장 가까운 경로점 찾기
            closest_idx = min(
                range(num_points), 
                key=lambda i: math.hypot(path_points[i][0] - wp_x, path_points[i][1] - wp_y)
            )
            
            # 웨이포인트 타입 및 속도 설정 적용
            if len(waypoint) > 3:
                target_speed = waypoint[3]
                
                if len(waypoint) > 2 and waypoint[2] in [2, 4]:  # 정지 지점
                    wp_vel = 0.0
                elif target_speed < 0:  # 최대 속도
                    wp_vel = self.max_speed
                else:
                    wp_vel = np.clip(target_speed, self.min_speed, self.max_speed)
            else:
                wp_vel = self.max_speed
            
            updated_limits[closest_idx] = min(updated_limits[closest_idx], wp_vel)
        
        return updated_limits
    
    def _generate_trapezoidal_profile(self, velocity_limits: List[float], 
                                    distances: List[float], start_vel: float, 
                                    end_vel: float) -> List[float]:
        """
        사다리꼴 속도 프로파일 생성 (가속도 제한 적용)
        
        Args:
            velocity_limits: 속도 제한 목록
            distances: 누적 거리 목록
            start_vel: 시작 속도
            end_vel: 종료 속도
            
        Returns:
            사다리꼴 속도 프로파일
        """
        num_points = len(velocity_limits)
        profile = list(velocity_limits)
        
        # 전진 패스: 가속도 제한 적용
        profile[0] = float(start_vel)
        for i in range(1, num_points):
            ds = distances[i] - distances[i-1]
            if ds < 1e-6:
                profile[i] = profile[i-1]
                continue
            
            # 가속도 제한하에서 도달 가능한 최대 속도
            max_reachable_vel = math.sqrt(profile[i-1]**2 + 2 * self.max_accel * ds)
            profile[i] = min(profile[i], max_reachable_vel)
        
        # 후진 패스: 감속도 제한 적용
        profile[-1] = float(end_vel)
        for i in range(num_points - 2, -1, -1):
            ds = distances[i+1] - distances[i]
            if ds < 1e-6:
                profile[i] = profile[i+1]
                continue
            
            # 감속도 제한하에서 도달 가능한 최대 속도
            max_braking_vel = math.sqrt(profile[i+1]**2 + 2 * self.max_decel * ds)
            profile[i] = min(profile[i], max_braking_vel)
        
        return profile
    
    def _apply_scurve_smoothing(self, trapezoidal_profile: List[float], 
                               distances: List[float]) -> List[float]:
        """
        S-Curve 스무딩 적용 (저크 제한)
        
        Args:
            trapezoidal_profile: 사다리꼴 속도 프로파일
            distances: 누적 거리 목록
            
        Returns:
            S-Curve 속도 프로파일
        """
        num_points = len(trapezoidal_profile)
        final_profile = list(trapezoidal_profile)
        current_accel = 0.0
        
        for i in range(1, num_points):
            ds = distances[i] - distances[i-1]
            if ds < 1e-6:
                final_profile[i] = final_profile[i-1]
                continue
            
            v_prev = final_profile[i-1]
            v_guide = trapezoidal_profile[i]
            
            # 목표 가속도 계산
            req_accel = (v_guide**2 - v_prev**2) / (2 * ds) if ds > 1e-6 else 0.0
            lim_accel = np.clip(req_accel, -self.max_decel, self.max_accel)
            
            # 저크 제한 적용
            dt = ds / max(v_prev, 0.1)
            max_accel_change = self.max_jerk * dt
            final_accel = np.clip(
                lim_accel, 
                current_accel - max_accel_change, 
                current_accel + max_accel_change
            )
            
            # 속도 업데이트
            v_from_jerk = math.sqrt(max(0, v_prev**2 + 2 * final_accel * ds))
            final_profile[i] = min(v_from_jerk, trapezoidal_profile[i])
            
            # 실제 가속도 업데이트
            current_accel = (final_profile[i]**2 - v_prev**2) / (2 * ds) if ds > 1e-6 else 0.0
        
        # 종료 속도 보장
        final_profile[-1] = trapezoidal_profile[-1]
        
        return final_profile 