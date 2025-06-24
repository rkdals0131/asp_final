#!/usr/bin/env python3
"""
Waypoint Parser Utility
웨이포인트 CSV 파일을 파싱하는 유틸리티 함수들을 제공

CSV 형식: x, y, mission_state, target_speed
- x: X 좌표 (float)
- y: Y 좌표 (float) 
- mission_state: 미션 상태 (int)
  - 1: 일반 웨이포인트
  - 2: 드론 이륙 지점 (정지)
  - 3: 경유지
  - 4: 최종 목적지 (정지)
- target_speed: 목표 속도 (float)
  - -1.0: 최대 속도
  - 0.0: 정지
  - 양수: 지정 속도 (m/s)
"""

import csv
import os
from typing import List, Tuple, Optional
import logging

class WaypointData:
    """웨이포인트 데이터 클래스"""
    def __init__(self, x: float, y: float, mission_state: int = 1, target_speed: float = -1.0):
        self.x = x
        self.y = y
        self.mission_state = mission_state
        self.target_speed = target_speed
    
    def __str__(self):
        return f"WP({self.x:.2f}, {self.y:.2f}, M{self.mission_state}, V{self.target_speed})"
    
    def to_tuple(self) -> Tuple[float, float, int, float]:
        """튜플 형태로 반환 (x, y, mission_state, target_speed)"""
        return (self.x, self.y, self.mission_state, self.target_speed)
    
    def to_ros_format(self) -> Tuple[float, float, int, float]:
        """ROS 파라미터 형태로 반환 (기존 path_follower_node와 호환)"""
        return self.to_tuple()

class WaypointParser:
    """웨이포인트 CSV 파서 클래스"""
    
    @staticmethod
    def parse_csv_file(file_path: str) -> List[WaypointData]:
        """
        CSV 파일에서 웨이포인트를 파싱
        
        Args:
            file_path: CSV 파일 경로
            
        Returns:
            WaypointData 객체들의 리스트
            
        Raises:
            FileNotFoundError: 파일이 존재하지 않을 때
            ValueError: CSV 형식이 올바르지 않을 때
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"웨이포인트 파일을 찾을 수 없음: {file_path}")
        
        waypoints = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                
                for row_idx, row in enumerate(reader):
                    # 빈 줄 또는 주석 줄 건너뛰기
                    if not row or len(row) == 0 or row[0].strip().startswith('#'):
                        continue
                    
                    # 최소 2개 값 (x, y) 필요
                    if len(row) < 2:
                        logging.warning(f"줄 {row_idx + 1}: 최소 2개 값(x, y)이 필요. 건너뜀")
                        continue
                    
                    try:
                        # 좌표 파싱
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        
                        # 미션 상태 파싱 (기본값: 1)
                        mission_state = 1
                        if len(row) > 2 and row[2].strip():
                            mission_state = int(row[2].strip())
                        
                        # 목표 속도 파싱 (기본값: -1.0)
                        target_speed = -1.0
                        if len(row) > 3 and row[3].strip():
                            target_speed = float(row[3].strip())
                        
                        waypoint = WaypointData(x, y, mission_state, target_speed)
                        waypoints.append(waypoint)
                        
                    except (ValueError, IndexError) as e:
                        logging.warning(f"줄 {row_idx + 1}: 파싱 오류 - {e}. 건너뜁니다.")
                        continue
                        
        except Exception as e:
            raise ValueError(f"CSV 파일 읽기 오류: {e}")
        
        if not waypoints:
            raise ValueError("유효한 웨이포인트가 없음")
        
        logging.info(f"총 {len(waypoints)}개의 웨이포인트를 로드했음")
        return waypoints
    
    @staticmethod
    def waypoints_to_ros_params(waypoints: List[WaypointData]) -> dict:
        """
        웨이포인트 리스트를 ROS 파라미터 형태로 변환
        
        Args:
            waypoints: WaypointData 객체들의 리스트
            
        Returns:
            ROS 파라미터 딕셔너리
        """
        if not waypoints:
            return {}
        
        # 평면 좌표 배열 생성 [x1, y1, x2, y2, ...]
        coords = []
        mission_types = []
        target_speeds = []
        waypoint_names = []
        
        for i, wp in enumerate(waypoints):
            coords.extend([wp.x, wp.y])
            mission_types.append(wp.mission_state)
            target_speeds.append(wp.target_speed)
            waypoint_names.append(f"wp_{i:02d}")
        
        return {
            'waypoints': coords,
            'mission_types': mission_types,
            'target_speeds': target_speeds,
            'waypoint_names': waypoint_names
        }
    
    @staticmethod
    def validate_waypoints(waypoints: List[WaypointData]) -> bool:
        """
        웨이포인트 데이터의 유효성을 검사
        
        Args:
            waypoints: 검사할 웨이포인트 리스트
            
        Returns:
            유효성 여부
        """
        if not waypoints:
            logging.error("웨이포인트가 비어있음")
            return False
        
        valid = True
        
        for i, wp in enumerate(waypoints):
            # 미션 상태 범위 체크
            if wp.mission_state not in [1, 2, 3, 4]:
                logging.warning(f"웨이포인트 {i}: 유효하지 않은 미션 상태 {wp.mission_state}")
                valid = False
            
            # 목표 속도 범위 체크
            if wp.target_speed < -1.0 or wp.target_speed > 10.0:
                logging.warning(f"웨이포인트 {i}: 유효하지 않은 목표 속도 {wp.target_speed}")
                valid = False
        
        return valid
    
    @staticmethod
    def get_mission_summary(waypoints: List[WaypointData]) -> dict:
        """
        미션 요약 정보를 반환
        
        Args:
            waypoints: 웨이포인트 리스트
            
        Returns:
            미션 요약 정보 딕셔너리
        """
        if not waypoints:
            return {}
        
        summary = {
            'total_waypoints': len(waypoints),
            'takeoff_points': 0,
            'waypoints': 0,
            'destination_points': 0,
            'stop_points': 0
        }
        
        for wp in waypoints:
            if wp.mission_state == 2:
                summary['takeoff_points'] += 1
            elif wp.mission_state == 3:
                summary['waypoints'] += 1
            elif wp.mission_state == 4:
                summary['destination_points'] += 1
            
            if wp.target_speed == 0.0:
                summary['stop_points'] += 1
        
        return summary

def load_waypoints_from_csv(file_path: str) -> List[WaypointData]:
    """
    편의 함수: CSV 파일에서 웨이포인트를 로드
    
    Args:
        file_path: CSV 파일 경로
        
    Returns:
        WaypointData 객체들의 리스트
    """
    parser = WaypointParser()
    return parser.parse_csv_file(file_path)

def waypoints_to_ros_params(waypoints: List[WaypointData]) -> dict:
    """
    편의 함수: 웨이포인트를 ROS 파라미터로 변환
    
    Args:
        waypoints: WaypointData 객체들의 리스트
        
    Returns:
        ROS 파라미터 딕셔너리
    """
    parser = WaypointParser()
    return parser.waypoints_to_ros_params(waypoints)

# 사용 예시
if __name__ == "__main__":
    # 테스트용 코드
    import sys
    
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        try:
            waypoints = load_waypoints_from_csv(csv_file)
            print(f"로드된 웨이포인트: {len(waypoints)}개")
            
            for i, wp in enumerate(waypoints):
                print(f"  {i}: {wp}")
            
            # 미션 요약
            parser = WaypointParser()
            summary = parser.get_mission_summary(waypoints)
            print(f"\n미션 요약: {summary}")
            
            # ROS 파라미터 형태로 변환
            ros_params = waypoints_to_ros_params(waypoints)
            print(f"\nROS 파라미터 형태:")
            for key, value in ros_params.items():
                print(f"  {key}: {value}")
                
        except Exception as e:
            print(f"오류: {e}")
    else:
        print("사용법: python waypoint_parser.py <csv_file_path>") 