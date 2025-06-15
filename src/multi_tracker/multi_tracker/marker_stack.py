#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
import csv
import os
import atexit

class MarkerStackNode(Node):
    """
    /marker_detections 토픽을 구독하여 마커 정보를 저장하고 시각화하며, 종료 시 CSV 파일로 저장합니다.
    """
    def __init__(self):
        super().__init__('marker_stack_node')
        
        # 내부 저장소: {marker_id: pose}
        self.stored_markers = {}
        
        # 구독자: /marker_detections 토픽
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/marker_detections',
            self.detection_callback,
            10)
            
        # 발행자: RViz2 시각화를 위한 MarkerArray
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # CSV 저장 경로 설정 및 디렉토리 생성
        # __file__은 현재 스크립트의 경로를 나타냅니다.
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.result_dir = os.path.join(current_dir, 'result')
        os.makedirs(self.result_dir, exist_ok=True)
        self.csv_path = os.path.join(self.result_dir, 'markers.csv')

        # 노드 종료 시 CSV 파일 저장을 위해 atexit에 등록
        atexit.register(self.save_to_csv)
        self.get_logger().info('MarkerStackNode has been started.')
        self.get_logger().info(f"Marker data will be saved to '{self.csv_path}' on shutdown.")


    def detection_callback(self, msg: Detection3DArray):
        """
        /marker_detections 토픽 메시지를 처리합니다.
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

        self.publish_markers(current_marker_ids)

    def publish_markers(self, current_marker_ids):
        """
        저장된 마커 정보를 기반으로 MarkerArray를 발행합니다.
        """
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 저장된 모든 마커에 대해 시각화 마커 생성
        for marker_id, pose in self.stored_markers.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "stored_markers"
            marker.id = int(marker_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose = pose
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.05
            
            # 현재 감지된 마커는 초록색, 나머지는 흰색
            if marker_id in current_marker_ids:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            marker.color.a = 1.0 # Alpha (불투명)
            
            marker_array.markers.append(marker)
            
        self.marker_publisher.publish(marker_array)

    def save_to_csv(self):
        """
        노드 종료 시 저장된 마커 정보를 CSV 파일로 저장합니다.
        """
        if not self.stored_markers:
            self.get_logger().info("No markers to save.")
            return
            
        self.get_logger().info(f"Saving {len(self.stored_markers)} markers to CSV file...")
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
            self.get_logger().info(f"Successfully saved markers to {self.csv_path}")
        except IOError as e:
            self.get_logger().error(f"Failed to write to CSV file: {e}")


def main(args=None):
    rclpy.init(args=args)
    marker_stack_node = MarkerStackNode()
    try:
        rclpy.spin(marker_stack_node)
    except KeyboardInterrupt:
        marker_stack_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # 노드 종료 시 save_to_csv가 atexit에 의해 호출됩니다.
        marker_stack_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
