#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray
import csv
import os
import atexit
from std_msgs.msg import Header

from robot_control.utils import viz_factory

class DetectedMarkerVisualizer(Node):
    """
    /marker_detections 토픽을 구독하여 탐지된 마커를 시각화하고,
    탐지된 마커의 최종 위치를 CSV 파일로 저장.
    """
    def __init__(self):
        super().__init__('detected_marker_visualizer')
        
        # 내부 저장소
        self.stored_markers = {}  # {marker_id_str: Pose}
        
        # 구독자: /marker_detections 토픽
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/marker_detections',
            self.detection_callback,
            10)
            
        # 발행자: RViz2 시각화를 위한 MarkerArray
        self.marker_publisher = self.create_publisher(MarkerArray, '/detected_markers_viz', 10)

        # CSV 저장 경로 설정
        current_dir = os.path.dirname(os.path.realpath(__file__))
        self.result_dir = os.path.join(current_dir, '..', 'utils', 'result')
        os.makedirs(self.result_dir, exist_ok=True)
        self.csv_path = os.path.join(self.result_dir, 'detected_markers.csv')

        # 노드 종료 시 CSV 파일 저장을 위해 atexit에 등록
        atexit.register(self.save_to_csv)
        self.get_logger().info('DetectedMarkerVisualizer has been started.')
        self.get_logger().info(f"Detected marker data will be saved to '{self.csv_path}' on shutdown.")


    def detection_callback(self, msg: Detection3DArray):
        """/marker_detections 토픽 메시지 처리."""
        current_marker_ids = set()
        
        for detection in msg.detections:
            if not detection.results: continue
            
            marker_id = detection.results[0].hypothesis.class_id
            pose = detection.results[0].pose.pose
            
            try:
                if not (0 <= int(marker_id) <= 6):
                    self.get_logger().warn(f"Invalid marker ID '{marker_id}'. Skipping.")
                    continue
            except ValueError:
                self.get_logger().warn(f"Non-integer marker ID '{marker_id}'. Skipping.")
                continue

            self.stored_markers[marker_id] = pose
            current_marker_ids.add(marker_id)

        self.publish_markers(current_marker_ids)

    def publish_markers(self, current_marker_ids: set):
        """
        저장된 탐지 마커 정보를 MarkerArray로 변환하여 발행.
        """
        header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        
        # viz_factory를 사용하여 마커 생성
        detected_markers = viz_factory.create_detected_markers(header, self.stored_markers, current_marker_ids)
        
        self.marker_publisher.publish(detected_markers)


    def save_to_csv(self):
        """노드 종료 시 탐지된 마커 정보를 CSV 파일로 저장."""
        if not self.stored_markers:
            self.get_logger().info("No detected markers to save.")
            return
            
        self.get_logger().info(f"Saving {len(self.stored_markers)} detected markers to CSV file...")
        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                fieldnames = ['marker_id', 'pos_x', 'pos_y', 'pos_z']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                writer.writeheader()
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
    visualizer_node = DetectedMarkerVisualizer()
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        visualizer_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        visualizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 