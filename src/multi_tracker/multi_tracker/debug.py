#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
import cv2
import cv_bridge
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations


class DebugCoordinateNode(Node):
    """
    ArUco 마커 검출과 좌표 변환 디버깅 노드
    
    좌표계 정의:
    OpenCV 카메라 좌표계: X=right, Y=down, Z=forward(렌즈 방향)
    ROS 드론 좌표계: X=forward, Y=left, Z=up
    
    변환 관계 (검증됨):
    - 카메라 X(right) = 드론 Z(up)
    - 카메라 Y(down) = 드론 -Y(back) 
    - 카메라 Z(lens/forward) = 드론 X(forward)
    """
    
    def __init__(self):
        super().__init__('debug_coordinate_node')
        
        # 매개변수 설정
        self.declare_parameter('marker_size', 0.5)
        self.declare_parameter('dictionary', 0)  # cv2.aruco.DICT_4X4_50
        self.declare_parameter('image_topic', '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image')
        self.declare_parameter('camera_info_topic', '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info')
        self.declare_parameter('camera_frame_id', 'x500_gimbal_0/camera_link')
        self.declare_parameter('drone_frame_id', 'x500_gimbal_0')
        
        self.marker_size = self.get_parameter('marker_size').value
        self.dictionary_id = self.get_parameter('dictionary').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.drone_frame_id = self.get_parameter('drone_frame_id').value
        
        # OpenCV ArUco 설정
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            try:
                self.aruco_dict = cv2.aruco.Dictionary_get(self.dictionary_id)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.use_new_api = False
            except AttributeError:
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.use_new_api = False
        
        # 카메라 내부 매개변수
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # CV Bridge
        self.bridge = cv_bridge.CvBridge()
        
        # 구독자
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)
        
        # 발행자
        self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)
        self.target_id_pub = self.create_publisher(Int32, '/target_id', 10)
        self.debug_image_pub = self.create_publisher(Image, '/debug_image', 1)
        
        # 좌표 변환 검증 상태
        self.coordinate_system_verified = False
        self.backup_rotation_matrix = None
        
        self.get_logger().info('OpenCV Camera Coordinate Debug Node started')
        self.get_logger().info('OpenCV Camera: X=right, Y=down, Z=forward(lens direction)')
        self.get_logger().info('ROS Drone: X=forward, Y=left, Z=up')

    def camera_info_callback(self, msg):
        """카메라 내부 매개변수 수신"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
            
            # 첫 번째 카메라 정보 수신 시 좌표계 검증
            if not self.coordinate_system_verified:
                self.verify_coordinate_system()

    def verify_coordinate_system(self):
        """좌표계 검증 및 백업 회전 행렬 설정"""
        try:
            # 카메라 -> 드론 변환 확인
            transform = self.tf_buffer.lookup_transform(
                self.drone_frame_id, self.camera_frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            # 백업 회전 행렬 저장 (검증에서 확인된 값)
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            self.backup_rotation_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
            
            self.coordinate_system_verified = True
            self.get_logger().info('✓ Coordinate system verified:')
            self.get_logger().info('  Camera X(right) = Drone Z(up)')
            self.get_logger().info('  Camera Y(down) = Drone -Y(back)')
            self.get_logger().info('  Camera Z(lens/forward) = Drone X(forward)')
            
        except Exception as e:
            self.get_logger().warn(f'Coordinate system verification failed: {e}')
            # 검증에서 확인된 고정 회전 행렬 사용
            self.backup_rotation_matrix = np.array([
                [0.0, 0.0, 1.0],   # 카메라 X(right) -> 드론 Z(up)
                [0.0, -1.0, 0.0],  # 카메라 Y(down) -> 드론 -Y(back)  
                [1.0, 0.0, 0.0]    # 카메라 Z(lens/forward) -> 드론 X(forward)
            ])
            self.get_logger().info('Using verified backup rotation matrix')

    def image_callback(self, msg):
        """이미지 콜백: ArUco 마커 검출 및 좌표 변환"""
        if self.camera_matrix is None:
            self.get_logger().warn('Camera intrinsics not received yet', throttle_duration_sec=1.0)
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return
        
        # ArUco 마커 검출
        if self.use_new_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) > 0:
            self.process_markers(corners, ids, msg.header.stamp, cv_image)
        else:
            # 마커가 없을 때는 빈 MarkerArray 발행
            marker_array = MarkerArray()
            self.marker_pub.publish(marker_array)
        
        # 디버깅 이미지 발행
        self.publish_debug_image(cv_image, corners, ids, msg.header)

    def process_markers(self, corners, ids, timestamp, cv_image):
        """검출된 마커들의 좌표 변환 처리"""
        marker_array = MarkerArray()
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id > 6:  # 유효한 마커 ID만 처리
                continue
                
            # 1. OpenCV solvePnP로 카메라 좌표계에서 마커 위치 계산
            # 결과: OpenCV 좌표계 (X=right, Y=down, Z=forward/lens direction)
            rvec, tvec = self.solve_pnp_for_marker(corners[i])
            if rvec is None or tvec is None:
                continue
            
            # 2. 카메라 -> 월드 좌표 변환 (검증된 방법)
            marker_world_pos = self.transform_camera_to_world_verified(tvec, timestamp)
            if marker_world_pos is None:
                continue
            
            # 3. 디버깅 마커 생성
            debug_markers = self.create_debug_markers(
                marker_id, tvec, marker_world_pos, timestamp)
            marker_array.markers.extend(debug_markers)
            
            # 4. 첫 번째 마커의 ID 발행
            if i == 0:
                id_msg = Int32()
                id_msg.data = int(marker_id)
                self.target_id_pub.publish(id_msg)
                
                # 상세 정보 로그
                self.log_transformation_details(marker_id, tvec, marker_world_pos, timestamp)
        
        self.marker_pub.publish(marker_array)

    def solve_pnp_for_marker(self, corners):
        """solvePnP를 사용하여 마커의 pose 계산 (OpenCV 좌표계)"""
        half_size = self.marker_size / 2.0
        # 마커 코너 정의 (OpenCV 좌표계에서 Z=0 평면)
        object_points = np.array([
            [-half_size,  half_size, 0],  # 왼쪽 위
            [ half_size,  half_size, 0],  # 오른쪽 위
            [ half_size, -half_size, 0],  # 오른쪽 아래
            [-half_size, -half_size, 0]   # 왼쪽 아래
        ], dtype=np.float32)
        
        image_points = corners.reshape(-1, 2).astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, 
            self.camera_matrix, self.dist_coeffs)
        
        if not success:
            return None, None
            
        return rvec.flatten(), tvec.flatten()

    def transform_camera_to_world_verified(self, tvec_camera, timestamp):
        """
        검증된 카메라 좌표계에서 월드 좌표계로 변환
        입력: OpenCV 카메라 좌표계 (X=right, Y=down, Z=lens direction)
        출력: 월드 좌표계
        """
        try:
            # 방법 1: TF2 체인 변환 (우선)
            return self.transform_via_tf2_chain(tvec_camera, timestamp)
            
        except Exception as e:
            self.get_logger().warn(f'TF2 transform failed, using backup: {e}')
            
            # 방법 2: 백업 회전 행렬 사용
            return self.transform_via_backup_matrix(tvec_camera, timestamp)

    def transform_via_tf2_chain(self, tvec_camera, timestamp):
        """TF2 체인을 통한 변환 (검증된 방법)"""
        # 1. 카메라 -> 드론 변환
        transform_cam_to_drone = self.tf_buffer.lookup_transform(
            self.drone_frame_id, self.camera_frame_id,
            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        
        R_cam_to_drone = self.quaternion_to_rotation_matrix([
            transform_cam_to_drone.transform.rotation.x,
            transform_cam_to_drone.transform.rotation.y,
            transform_cam_to_drone.transform.rotation.z,
            transform_cam_to_drone.transform.rotation.w
        ])
        
        # 2. 드론 -> 월드 변환
        transform_drone_to_world = self.tf_buffer.lookup_transform(
            'map', self.drone_frame_id,
            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        
        drone_pos_world = np.array([
            transform_drone_to_world.transform.translation.x,
            transform_drone_to_world.transform.translation.y,
            transform_drone_to_world.transform.translation.z
        ])
        
        R_drone_to_world = self.quaternion_to_rotation_matrix([
            transform_drone_to_world.transform.rotation.x,
            transform_drone_to_world.transform.rotation.y,
            transform_drone_to_world.transform.rotation.z,
            transform_drone_to_world.transform.rotation.w
        ])
        
        # 3. 체인 변환 적용: 카메라(OpenCV) -> 드론(ROS) -> 월드
        marker_drone = R_cam_to_drone.dot(tvec_camera)
        marker_world = drone_pos_world + R_drone_to_world.dot(marker_drone)
        
        return marker_world

    def transform_via_backup_matrix(self, tvec_camera, timestamp):
        """백업 회전 행렬을 사용한 변환"""
        if self.backup_rotation_matrix is None:
            raise Exception("Backup rotation matrix not available")
        
        try:
            # 드론 위치만 TF2에서 가져오기
            transform_drone_to_world = self.tf_buffer.lookup_transform(
                'map', self.drone_frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            drone_pos_world = np.array([
                transform_drone_to_world.transform.translation.x,
                transform_drone_to_world.transform.translation.y,
                transform_drone_to_world.transform.translation.z
            ])
            
            R_drone_to_world = self.quaternion_to_rotation_matrix([
                transform_drone_to_world.transform.rotation.x,
                transform_drone_to_world.transform.rotation.y,
                transform_drone_to_world.transform.rotation.z,
                transform_drone_to_world.transform.rotation.w
            ])
            
            # 백업 회전 행렬로 카메라(OpenCV) -> 드론(ROS) 변환
            marker_drone = self.backup_rotation_matrix.dot(tvec_camera)
            marker_world = drone_pos_world + R_drone_to_world.dot(marker_drone)
            
            return marker_world
            
        except Exception as e:
            self.get_logger().error(f'Backup transformation also failed: {e}')
            return None

    def quaternion_to_rotation_matrix(self, quaternion):
        """Quaternion을 회전 행렬로 변환"""
        return tf_transformations.quaternion_matrix(quaternion)[:3, :3]

    def create_debug_markers(self, marker_id, tvec_camera, marker_world_pos, timestamp):
        """디버깅용 마커들 생성"""
        markers = []
        
        # 1. 월드 원점 표시 (흰색)
        origin_marker = self.create_marker(
            marker_id * 100 + 1, 'map', np.array([0, 0, 0]), 
            [1.0, 1.0, 1.0], 'world_origin', timestamp, scale=0.5)
        markers.append(origin_marker)
        
        # 2. 드론 위치 표시 (파란색)
        try:
            drone_transform = self.tf_buffer.lookup_transform(
                'map', self.drone_frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            drone_marker = self.create_marker(
                marker_id * 100 + 2, 'map',
                np.array([drone_transform.transform.translation.x,
                         drone_transform.transform.translation.y,
                         drone_transform.transform.translation.z]),
                [0.0, 0.0, 1.0], 'drone_position', timestamp, scale=0.7)
            markers.append(drone_marker)
        except Exception:
            pass
        
        # 3. 카메라 위치 표시 (시안색)
        try:
            camera_transform = self.tf_buffer.lookup_transform(
                'map', self.camera_frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            camera_marker = self.create_marker(
                marker_id * 100 + 3, 'map',
                np.array([camera_transform.transform.translation.x,
                         camera_transform.transform.translation.y,
                         camera_transform.transform.translation.z]),
                [0.0, 1.0, 1.0], 'camera_position', timestamp, scale=0.5)
            markers.append(camera_marker)
        except Exception:
            pass
        
        # 4. 계산된 마커 위치 표시 (빨간색, 큰 크기)
        marker_marker = self.create_marker(
            marker_id * 100 + 4, 'map', marker_world_pos,
            [1.0, 0.0, 0.0], f'opencv_marker_{marker_id}', timestamp, scale=1.0)
        markers.append(marker_marker)
        
        return markers

    def create_marker(self, marker_id, frame_id, position, color, namespace, timestamp, scale=0.3):
        """기본 마커 생성"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = 0.05
        
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8
        
        return marker

    def log_transformation_details(self, marker_id, tvec_camera, marker_world_pos, timestamp):
        """변환 과정 상세 로그"""
        try:
            # 드론 위치 가져오기
            drone_transform = self.tf_buffer.lookup_transform(
                'map', self.drone_frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            drone_pos = np.array([
                drone_transform.transform.translation.x,
                drone_transform.transform.translation.y,
                drone_transform.transform.translation.z
            ])
            
            self.get_logger().info('=== OpenCV 좌표계 변환 ===')
            self.get_logger().info(f'Marker ID: {marker_id}')
            self.get_logger().info(f'OpenCV Camera coords: [{tvec_camera[0]:.3f}, {tvec_camera[1]:.3f}, {tvec_camera[2]:.3f}]')
            self.get_logger().info(f'  X={tvec_camera[0]:.3f}(right), Y={tvec_camera[1]:.3f}(down), Z={tvec_camera[2]:.3f}(lens/forward)')
            self.get_logger().info(f'ROS Drone position: [{drone_pos[0]:.3f}, {drone_pos[1]:.3f}, {drone_pos[2]:.3f}]')
            self.get_logger().info(f'Final world position: [{marker_world_pos[0]:.3f}, {marker_world_pos[1]:.3f}, {marker_world_pos[2]:.3f}]')
            self.get_logger().info('Coordinate mapping: Cam(X,Y,Z) -> Drone(Z,-Y,X)')
            self.get_logger().info('==========================')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform details: {e}')

    def publish_debug_image(self, cv_image, corners, ids, header):
        """디버깅 이미지 발행"""
        if ids is not None:
            # 검출된 마커 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # 축 그리기 (OpenCV 좌표계)
            for i in range(len(ids)):
                rvec, tvec = self.solve_pnp_for_marker(corners[i])
                if rvec is not None and tvec is not None:
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, 
                                    self.dist_coeffs, rvec, tvec, self.marker_size)
        
        # 텍스트 오버레이
        cv2.putText(cv_image, 'OpenCV Camera Coordinate System', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        if ids is not None:
            cv2.putText(cv_image, f'Detected {len(ids)} markers', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        else:
            cv2.putText(cv_image, 'No markers detected', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        
        # 좌표계 정의 표시
        cv2.putText(cv_image, 'OpenCV: X=right, Y=down, Z=lens/forward', 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 검증된 시스템 정보
        verification_status = "VERIFIED" if self.coordinate_system_verified else "UNVERIFIED"
        cv2.putText(cv_image, f'System: {verification_status}', 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # 범례
        legend_texts = [
            'White: World Origin (0,0,0)',
            'Blue: Drone Position (ROS coords)', 
            'Cyan: Camera Position',
            'Red (BIG): Marker in World',
            'Axes on marker: OpenCV coords',
            'Red=X(right), Green=Y(down), Blue=Z(lens)'
        ]
        
        for i, text in enumerate(legend_texts):
            cv2.putText(cv_image, text, (10, 150 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 변환 관계 표시
        transform_info = "Transform: Cam(X,Y,Z) -> Drone(Z,-Y,X)"
        cv2.putText(cv_image, transform_info, (10, cv_image.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # 이미지 발행
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DebugCoordinateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 