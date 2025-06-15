#include "multi_tracker/multi_tracker_node.hpp"
#include <sstream>
#include <iomanip>

// TF2 관련 헤더
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

    _target_id_pub = this->create_publisher<std_msgs::msg::Int32>(_target_id_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
    _marker_detections_pub = this->create_publisher<vision_msgs::msg::Detection3DArray>(_marker_detections_topic, pose_qos);
}

void MultiTrackerNode::loadParameters()
{
    declare_parameter<int>("dictionary", 0);
    declare_parameter<double>("marker_size", 0.5);
    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);

    declare_parameter<std::string>("image_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    declare_parameter<std::string>("camera_info_topic", "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info");
    declare_parameter<std::string>("camera_frame_id", "x500_gimbal_0/camera_link");
    
    declare_parameter<std::string>("target_id_topic", "/target_id");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    declare_parameter<std::string>("marker_detections_topic", "/marker_detections");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    get_parameter("camera_frame_id", _camera_frame_id);
    get_parameter("target_id_topic", _target_id_topic);
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
        RCLCPP_WARN_ONCE(this->get_logger(), "Camera intrinsics are not initialized yet.");
        return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detectorParams);

    if (!ids.empty()) {
        // 실제 마커의 고정된 위치를 TF에서 가져오기
        geometry_msgs::msg::TransformStamped T_map_to_actual_marker_msg;
        try {
            // 실제 마커의 위치 (TF에서 고정된 값)
            T_map_to_actual_marker_msg = _tf_buffer->lookupTransform("map", "X1_asp/aruco_marker_6_link", rclcpp::Time(0));
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Actual marker position in TF: X=%.2f, Y=%.2f, Z=%.2f",
                T_map_to_actual_marker_msg.transform.translation.x,
                T_map_to_actual_marker_msg.transform.translation.y,
                T_map_to_actual_marker_msg.transform.translation.z);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Could not get actual marker position: %s", ex.what());
        }

        // 명시적인 TF lookup: map -> camera optical frame 직접 변환
        geometry_msgs::msg::TransformStamped T_map_to_optical_msg;
        std::string optical_frame_id = _camera_frame_id + "_optical_frame";
        
        try {
            // 시도 1: optical frame이 존재하는 경우
            T_map_to_optical_msg = _tf_buffer->lookupTransform("map", optical_frame_id, rclcpp::Time(0));
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Using direct optical frame: %s", optical_frame_id.c_str());
        } catch (const tf2::TransformException & ex) {
            try {
                // 시도 2: optical frame이 없는 경우, 물리적 카메라 프레임 사용
                T_map_to_optical_msg = _tf_buffer->lookupTransform("map", _camera_frame_id, rclcpp::Time(0));
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Using physical camera frame: %s (optical frame not found)", _camera_frame_id.c_str());
                
                // 물리적 카메라 프레임에서 광학 프레임으로의 변환 적용
                // 표준 ROS 카메라 광학 변환: X=right, Y=down, Z=forward
                tf2::Transform T_physical_to_optical;
                tf2::Matrix3x3 R_phys_to_opt(
                     0, -1,  0,   // X_optical = -Y_physical
                     0,  0, -1,   // Y_optical = -Z_physical  
                     1,  0,  0    // Z_optical =  X_physical
                );
                T_physical_to_optical.setBasis(R_phys_to_opt);
                T_physical_to_optical.setOrigin(tf2::Vector3(0, 0, 0));
                
                // 물리적 프레임 변환에 광학 변환 적용
                tf2::Transform T_map_to_physical;
                tf2::fromMsg(T_map_to_optical_msg.transform, T_map_to_physical);
                tf2::Transform T_map_to_optical_corrected = T_map_to_physical * T_physical_to_optical;
                T_map_to_optical_msg.transform = tf2::toMsg(T_map_to_optical_corrected);
                
            } catch (const tf2::TransformException & ex2) {
                RCLCPP_ERROR(this->get_logger(), "Could not find camera frame '%s' or '%s': %s", 
                           _camera_frame_id.c_str(), optical_frame_id.c_str(), ex2.what());
                return;
            }
        }
        
        // TF에서 가져온 변환을 tf2::Transform으로 변환
        tf2::Transform T_map_to_optical;
        tf2::fromMsg(T_map_to_optical_msg.transform, T_map_to_optical);
        
        // 디버깅: 카메라 광학 프레임 위치 출력
        tf2::Vector3 optical_pos = T_map_to_optical.getOrigin();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Camera optical frame in map: X=%.2f, Y=%.2f, Z=%.2f", 
            optical_pos.x(), optical_pos.y(), optical_pos.z());

        vision_msgs::msg::Detection3DArray detections_msg;
        detections_msg.header.stamp = msg->header.stamp;
        detections_msg.header.frame_id = "map";

        for (size_t i = 0; i < ids.size(); i++) {
            float half_size = _param_marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
            cv::Vec3d rvec, tvec;
            // solvePnP: 카메라 광학 좌표계 기준 마커의 상대적 위치와 자세
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            
            // 디버깅: solvePnP 결과 (광학 좌표계 기준)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Marker %d in camera optical frame: X=%.2f, Y=%.2f, Z=%.2f", 
                ids[i], tvec[0], tvec[1], tvec[2]);
            
            // solvePnP 결과를 tf2::Transform으로 변환
            tf2::Transform T_optical_to_marker;
            T_optical_to_marker.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));
            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            T_optical_to_marker.getBasis().setValue(
                R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
            );
            
            // 핵심: 명시적 TF 변환으로 map 프레임에서 마커의 절대 위치 계산
            // T_map_to_marker = T_map_to_optical * T_optical_to_marker
            tf2::Transform T_map_to_marker = T_map_to_optical * T_optical_to_marker;
            
            // 결과 디버깅
            tf2::Vector3 marker_pos_in_map = T_map_to_marker.getOrigin();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Marker %d absolute position in map: X=%.2f, Y=%.2f, Z=%.2f", 
                ids[i], marker_pos_in_map.x(), marker_pos_in_map.y(), marker_pos_in_map.z());
            
            vision_msgs::msg::Detection3D detection;
            detection.header = detections_msg.header;
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = std::to_string(ids[i]); 
            hypothesis.hypothesis.score = 1.0; 
            tf2::toMsg(T_map_to_marker, hypothesis.pose.pose);
            
            detection.results.push_back(hypothesis);
            detections_msg.detections.push_back(detection);

            std_msgs::msg::Int32 id_msg;
            id_msg.data = ids[i];
            _target_id_pub->publish(id_msg);
            
            if (i == 0) {
                _target[0] = hypothesis.pose.pose.position.x;
                _target[1] = hypothesis.pose.pose.position.y;
                _target[2] = hypothesis.pose.pose.position.z;
                
                // 최종 결과 로그
                RCLCPP_INFO(this->get_logger(), "Final Marker %d in map frame: X=%.2f, Y=%.2f, Z=%.2f", 
                           ids[i], _target[0], _target[1], _target[2]);
            }
        }

        if (!detections_msg.detections.empty()) {
            _marker_detections_pub->publish(detections_msg);
        }
    }
    
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
    if (!ids.empty()) {
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Vec3d rvec, tvec;
            float half_size = _param_marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
            // Note: This solvePnP is redundant if performance is critical, but useful for drawing axes.
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            // drawFrameAxes는 광학 좌표계 기준이므로 그대로 사용합니다.
            cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvec, tvec, _param_marker_size);
        }
    }
    
    annotate_image(cv_ptr, !ids.empty());
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = cv_ptr->image;
    _image_pub->publish(*out_msg.toImageMsg().get());
}

void MultiTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!_camera_matrix.empty()) {
        return;
    }
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Successfully received camera intrinsic parameters.");
}

void MultiTrackerNode::annotate_image(cv_bridge::CvImagePtr image, bool detected)
{
    if (detected) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2);
        stream << "World X:"  << _target[0] << " Y:" << _target[1]  << " Z:" << _target[2];
        std::string text_xyz = stream.str();
        cv::putText(image->image, text_xyz, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(image->image, "No marker detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}