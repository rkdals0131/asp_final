#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
import csv
import os
import atexit
import math

class MarkerVisualNode(Node):
    """
    /marker_detections 토픽을 구독하여 마커 정보를 저장하고 시각화하며, 
    Ground Truth 마커도 함께 시각화
    """
    def __init__(self):
        super().__init__('marker_visual_node')
        
        # 파라미터 선언
        self.declare_parameter('show_ground_truth', True)
        self.declare_parameter('ground_truth_csv_path', 'config/aruco_markers.csv')
        
        # 내부 저장소: {marker_id: pose}
        self.stored_markers = {}
        self.ground_truth_markers = {}
        
        # 파라미터 값 가져오기
        self.show_ground_truth = self.get_parameter('show_ground_truth').get_parameter_value().bool_value
        gt_csv_path = self.get_parameter('ground_truth_csv_path').get_parameter_value().string_value
        
        # 구독자: /marker_detections 토픽
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/marker_detections',
            self.detection_callback,
            10)
            
        # 발행자: RViz2 시각화를 위한 MarkerArray
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # CSV 저장 경로 설정 및 디렉토리 생성
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.result_dir = os.path.join(current_dir, 'result')
        os.makedirs(self.result_dir, exist_ok=True)
        self.csv_path = os.path.join(self.result_dir, 'detected_markers.csv')

        # Ground Truth 마커 로드
        self.load_ground_truth_markers(gt_csv_path)

        # 노드 종료 시 CSV 파일 저장을 위해 atexit에 등록
        atexit.register(self.save_to_csv)
        self.get_logger().info('MarkerVisualNode has been started.')
        self.get_logger().info(f"Ground Truth visualization: {'ON' if self.show_ground_truth else 'OFF'}")
        self.get_logger().info(f"Detected marker data will be saved to '{self.csv_path}' on shutdown.")

        # Ground Truth 마커를 즉시 발행 (파라미터가 True인 경우)
        if self.show_ground_truth:
            self.publish_ground_truth_markers()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
            오일러 각도(roll, pitch, yaw)를 쿼터니언으로 변환
    Extrinsic ZYX (Yaw-Pitch-Roll) 순서로 변환하여 적용
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Extrinsic ZYX (Yaw-Pitch-Roll)에 해당하는 변환 공식
        qw = cr * cp * cy - sr * sp * sy
        qx = sr * cp * cy + cr * sp * sy
        qy = cr * sp * cy - sr * cp * sy
        qz = cr * cp * sy + sr * sp * cy
        
        return qx, qy, qz, qw

    def load_ground_truth_markers(self, csv_path):
        """
            Ground Truth 마커 정보를 CSV 파일에서 로드
    CSV의 roll, pitch, yaw 값을 포함하여 전체 포즈를 사용
        """
        # 절대 경로 또는 상대 경로 처리
        if not os.path.isabs(csv_path):
            # robot_control 패키지의 config 디렉토리에서 찾기
            package_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
            csv_path = os.path.join(package_dir, csv_path)
        
        if not os.path.exists(csv_path):
            self.get_logger().warn(f"Ground Truth CSV file not found: {csv_path}")
            return
            
        try:
            with open(csv_path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    marker_name = row['name']
                    if marker_name.startswith('aruco_marker_'):
                        try:
                            # 'aruco_marker_4_1' -> '41'과 같이 고유 ID 생성
                            marker_id_str = marker_name.replace('aruco_marker_', '').replace('_', '')
                            marker_id = int(marker_id_str)

                            roll = float(row['roll'])
                            pitch = float(row['pitch'])
                            yaw = float(row['yaw'])
                            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

                            self.ground_truth_markers[marker_id] = {
                                'x': float(row['e']),
                                'y': float(row['n']),
                                'z': float(row['u']),
                                'qx': qx,
                                'qy': qy,
                                'qz': qz,
                                'qw': qw
                            }
                        except (ValueError, KeyError) as e:
                            self.get_logger().warn(f"Skipping row due to invalid data for marker '{marker_name}': {e}")
                            
            self.get_logger().info(f"Loaded {len(self.ground_truth_markers)} ground truth markers")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load ground truth markers: {e}")

    def detection_callback(self, msg: Detection3DArray):
        """
        /marker_detections 토픽 메시지를 처리
        """
        current_marker_ids = set()
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            # class_id를 마커 ID로 사용
            marker_id = detection.results[0].hypothesis.class_id
            pose = detection.results[0].pose.pose
            
            # ID가 0~6 사이의 정수인지 확인
            try:
                if not (0 <= int(marker_id) <= 6):
                    self.get_logger().warn(f"Invalid marker ID '{marker_id}' received. It must be between 0 and 6. Skipping.")
                    continue
            except ValueError:
                self.get_logger().warn(f"Marker ID '{marker_id}' is not an integer. Skipping.")
                continue

            # 마커 정보 저장 (덮어쓰기)
            self.stored_markers[marker_id] = pose
            current_marker_ids.add(marker_id)

        self.publish_all_markers(current_marker_ids)

    def publish_ground_truth_markers(self):
        """
        Ground Truth 마커만 발행
        """
        if not self.show_ground_truth:
            return
            
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for marker_id, position in self.ground_truth_markers.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "ground_truth_markers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 위치 및 회전 설정
            marker.pose.position.x = position['x']
            marker.pose.position.y = position['y']
            marker.pose.position.z = position['z']
            marker.pose.orientation.x = position['qx']
            marker.pose.orientation.y = position['qy']
            marker.pose.orientation.z = position['qz']
            marker.pose.orientation.w = position['qw']
            
            # Ground Truth 마커: 가로/세로 1.5m, 높이 0.3m sphere
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3  # 약간 투명하게
            
            marker_array.markers.append(marker)
            
        self.marker_publisher.publish(marker_array)

    def publish_all_markers(self, current_marker_ids):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Ground Truth 마커 추가 (파라미터가 True인 경우)
        if self.show_ground_truth:
            for marker_id, position in self.ground_truth_markers.items():
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = now
                marker.ns = "ground_truth_markers"
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # 위치 및 회전 설정
                marker.pose.position.x = position['x']
                marker.pose.position.y = position['y']
                marker.pose.position.z = position['z']
                marker.pose.orientation.x = position['qx']
                marker.pose.orientation.y = position['qy']
                marker.pose.orientation.z = position['qz']
                marker.pose.orientation.w = position['qw']
                
                # Ground Truth 마커: 가로/세로 1.5m, 높이 0.3m sphere
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.3  # 약간 투명하게
                
                marker_array.markers.append(marker)

        # 탐지된 마커 추가
        for marker_id, pose in self.stored_markers.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "detected_markers"
            marker.id = int(marker_id) + 100  # ID 충돌 방지를 위해 100 더함
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose = pose
            
            # 탐지된 마커: 0.5m x 0.5m x 0.05m 노란색 큐브
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.1
            
            # 현재 감지된 마커는 밝은 노란색, 이전에 감지된 마커는 어두운 노란색
            if marker_id in current_marker_ids:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.3
                marker.color.a = 1.0
            else:
                marker.color.r = 0.8
                marker.color.g = 0.8
                marker.color.b = 0.0
                marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
        self.marker_publisher.publish(marker_array)

    def save_to_csv(self):
        """
        노드 종료 시 탐지된 마커 정보를 CSV 파일로 저장
        """
        if not self.stored_markers:
            self.get_logger().info("No detected markers to save.")
            return
            
        self.get_logger().info(f"Saving {len(self.stored_markers)} detected markers to CSV file...")
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                fieldnames = ['marker_id', 'pos_x', 'pos_y', 'pos_z']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
                # ID를 기준으로 정렬하여 저장
                for marker_id in sorted(self.stored_markers.keys(), key=int):
                    pose = self.stored_markers[marker_id]
                    writer.writerow({
                        'marker_id': marker_id,
                        'pos_x': pose.position.x,
                        'pos_y': pose.position.y,
                        'pos_z': pose.position.z,
                    })
            self.get_logger().info(f"Successfully saved detected markers to {self.csv_path}")
        except IOError as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")


def main(args=None):
    rclpy.init(args=args)
    marker_visual_node = MarkerVisualNode()
    try:
        rclpy.spin(marker_visual_node)
    except KeyboardInterrupt:
        marker_visual_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # 노드 종료 시 save_to_csv가 atexit에 의해 호출됩니다.
        marker_visual_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 