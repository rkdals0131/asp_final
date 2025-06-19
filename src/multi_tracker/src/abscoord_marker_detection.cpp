#include "multi_tracker/abscoord_marker_detection.hpp"
#include <sstream>
#include <iomanip>

// TF2 and geometry_msgs
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

MultiTrackerNode::MultiTrackerNode()
    : Node("multi_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting MultiTrackerNode");

    loadParameters();

    auto image_qos = rclcpp::QoS(1).best_effort();
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    _detectorParams = cv::aruco::DetectorParameters::create();
    _dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        _image_topic, image_qos,
        std::bind(&MultiTrackerNode::image_callback, this, std::placeholders::_1));

    _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        _camera_info_topic, image_qos,
        std::bind(&MultiTrackerNode::camera_info_callback, this, std::placeholders::_1));

    _command_sub = this->create_subscription<std_msgs::msg::String>(
        "/multi_tracker/command", 10,
        std::bind(&MultiTrackerNode::command_callback, this, std::placeholders::_1));

    _marker_detections_pub = this->create_publisher<vision_msgs::msg::Detection3DArray>(_marker_detections_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
    
    RCLCPP_INFO(this->get_logger(), "MultiTrackerNode initialized with landing mode support");
}

void MultiTrackerNode::loadParameters()
{
    declare_parameter<int>("dictionary", 0);
    declare_parameter<double>("marker_size", 1.0);
    declare_parameter<double>("landing_marker_size", 0.5);
    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);
    get_parameter("landing_marker_size", _param_landing_marker_size);

    declare_parameter<std::string>("image_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    declare_parameter<std::string>("camera_info_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info");
    declare_parameter<std::string>("camera_frame_id", "x500_gimbal_0/camera_link_rdf");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    declare_parameter<std::string>("marker_detections_topic", "/marker_detections");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    get_parameter("camera_frame_id", _camera_frame_id);
    get_parameter("image_proc_topic", _image_proc_topic);
    get_parameter("marker_detections_topic", _marker_detections_topic);
    
    RCLCPP_INFO(this->get_logger(), "Camera frame ID: %s", _camera_frame_id.c_str());
}

void MultiTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        if (msg->encoding == "rgb8") {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (_camera_matrix.empty() || _dist_coeffs.empty()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Camera info is not ready yet.");
        return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detectorParams);

    if (!ids.empty()) {
        // Step 1: 'map' 좌표계에서 카메라(camera_rdf) 좌표계로의 변환(Transform)을 조회합니다.
        geometry_msgs::msg::TransformStamped T_map_from_camera_rdf_msg;
        tf2::Transform T_map_from_camera_rdf;
        try {
            const rclcpp::Time& timestamp = msg->header.stamp;
            const tf2::Duration timeout = tf2::durationFromSec(0.1);

            // Look up transform from map to camera RDF frame directly
            std::string camera_rdf_frame = _camera_frame_id;
            T_map_from_camera_rdf_msg = _tf_buffer->lookupTransform(
                "map", camera_rdf_frame, timestamp, timeout);
            tf2::fromMsg(T_map_from_camera_rdf_msg.transform, T_map_from_camera_rdf);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from 'map' -> '%s': %s",
                _camera_frame_id.c_str(), ex.what());
            return;
        }

        vision_msgs::msg::Detection3DArray detections_msg;
        detections_msg.header.stamp = msg->header.stamp;
        detections_msg.header.frame_id = "map"; // Final coordinates will be in the map frame.

        for (size_t i = 0; i < ids.size(); i++) { // 감지된 모든 마커에 대해
            // Step 2: Estimate the marker's pose using solvePnP (returns pose in camera's optical frame)
            // Use conditional marker size based on landing mode
            float marker_size = _is_landing_mode ? _param_landing_marker_size : _param_marker_size;
            float half_size = marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            }; // 마커의 중심을 원점으로 하는 네 꼭지점
            cv::Vec3d rvec, tvec;
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            
            // Step 3: solvePnP 결과는 이미 카메라의 RDF 좌표계 기준입니다.
            // 카메라의 RDF 좌표계를 기준으로 한 마커의 포즈(pose)를 나타내는 Transform을 생성합니다.
            // T_camera_rdf_from_marker: 마커 좌표계의 점을 카메라 RDF 좌표계로 변환합니다.
            tf2::Transform T_camera_rdf_from_marker;
            T_camera_rdf_from_marker.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));
            
            // 회전 벡터(rvec)를 tf2::Quaternion으로 변환합니다.
            cv::Mat rot_matrix;
            cv::Rodrigues(rvec, rot_matrix);
            tf2::Matrix3x3 tf2_rot_matrix(
                rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
                rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
                rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));
            tf2::Quaternion tf2_quat;
            tf2_rot_matrix.getRotation(tf2_quat);
            T_camera_rdf_from_marker.setRotation(tf2_quat);

            // 디버깅을 위해 변환 관계를 저장합니다.
            _T_camera_rdf_from_marker_map[ids[i]] = T_camera_rdf_from_marker;
            
            // Step 4: 'map' 좌표계에서의 최종 마커 포즈를 계산합니다.
            // 변환 연결: T_map_from_marker = T_map_from_camera * T_camera_from_marker
            tf2::Transform T_map_from_marker = T_map_from_camera_rdf * T_camera_rdf_from_marker;
            
            tf2::Vector3 marker_pos_in_map = T_map_from_marker.getOrigin();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "마커 %d가 맵 좌표 (X=%.2f, Y=%.2f, Z=%.2f)에서 감지됨",
                ids[i], marker_pos_in_map.x(), marker_pos_in_map.y(), marker_pos_in_map.z());
            
            // 추가적인 디버그 정보
            tf2::Vector3 marker_in_camera_rdf = T_camera_rdf_from_marker.getOrigin();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "마커 %d의 카메라 기준(camera_rdf) 좌표: X=%.3f, Y=%.3f, Z=%.3f",
                ids[i], marker_in_camera_rdf.x(), marker_in_camera_rdf.y(), marker_in_camera_rdf.z());
            
            // Step 5: 발행할 메시지를 채웁니다.
            vision_msgs::msg::Detection3D detection;
            detection.header = detections_msg.header;
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = std::to_string(ids[i]); 
            hypothesis.hypothesis.score = 1.0; 
            tf2::toMsg(T_map_from_marker, hypothesis.pose.pose);
            
            detection.results.push_back(hypothesis);
            detections_msg.detections.push_back(detection);
        }

        if (!detections_msg.detections.empty()) {
            _marker_detections_pub->publish(detections_msg);
        }
    }
    
    // 이미지 토픽에 시각화를 위해 마커와 좌표축을 그립니다.
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
    if (!ids.empty()) {
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Vec3d rvec, tvec;
            // Use conditional marker size based on landing mode
            float marker_size = _is_landing_mode ? _param_landing_marker_size : _param_marker_size;
            float half_size = marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvec, tvec, marker_size);
        }
    }
    
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = cv_ptr->image;
    _image_pub->publish(*out_msg.toImageMsg().get());
}

void MultiTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!_camera_matrix.empty()) {
        // Already received camera info
        return;
    }
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    RCLCPP_INFO_ONCE(this->get_logger(), "Successfully received camera intrinsic parameters.");
}

void MultiTrackerNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "DETECT_LANDING_MARKER") {
        RCLCPP_INFO(this->get_logger(), "Landing mode activated. Using landing marker size (%.2fm).", _param_landing_marker_size);
        _is_landing_mode = true;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
