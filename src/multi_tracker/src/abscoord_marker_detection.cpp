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

    // =================================================================================
    // Coordinate System Definitions & Static Transform
    // =================================================================================
    // 1. World Frame (map): ENU (East-North-Up)
    //    - x: East, y: North, z: Up
    //
    // 2. Camera Physical Frame (camera_link): Standard ROS convention
    //    - x: forward, y: left, z: up
    //
    // 3. Camera Optical Frame (OpenCV): Standard RDF (Right-Down-Forward)
    //    - x: right, y: down, z: forward (out of the lens)
    //
    // We need the static transform from the Optical frame to the Physical frame.
    // T_physical_from_optical
    //
    // Mapping:
    // Physical X (forward) <- Optical Z (forward)
    // Physical Y (left)    <- Optical -X (right)
    // Physical Z (up)      <- Optical -Y (down)
    //
    // The rotation matrix to transform a point from Optical to Physical is:
    // | 0  0  1 |
    // |-1  0  0 |
    // | 0 -1  0 |
    // =================================================================================
    tf2::Matrix3x3 rotation_physical_from_optical(
         0,  0,  1,
        -1,  0,  0,
         0, -1,  0
    );
    _T_physical_from_optical.setBasis(rotation_physical_from_optical);
    _T_physical_from_optical.setOrigin(tf2::Vector3(0, 0, 0));


    _image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        _image_topic, image_qos,
        std::bind(&MultiTrackerNode::image_callback, this, std::placeholders::_1));

    _camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        _camera_info_topic, image_qos,
        std::bind(&MultiTrackerNode::camera_info_callback, this, std::placeholders::_1));

    _marker_detections_pub = this->create_publisher<vision_msgs::msg::Detection3DArray>(_marker_detections_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
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
    declare_parameter<std::string>("drone_frame_id", "x500_gimbal_0");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    declare_parameter<std::string>("marker_detections_topic", "/marker_detections");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    get_parameter("camera_frame_id", _camera_frame_id);
    get_parameter("drone_frame_id", _drone_frame_id);
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
        // Step 1: Get the transforms from 'map' -> 'drone' and 'drone' -> camera's physical frame for debugging.
        geometry_msgs::msg::TransformStamped T_map_from_drone_msg, T_drone_from_physical_msg;
        tf2::Transform T_map_from_drone, T_drone_from_physical;
        try {
            const rclcpp::Time& timestamp = msg->header.stamp;
            const tf2::Duration timeout = tf2::durationFromSec(0.1);

            // Look up transform from map to drone
            T_map_from_drone_msg = _tf_buffer->lookupTransform(
                "map", _drone_frame_id, timestamp, timeout);
            tf2::fromMsg(T_map_from_drone_msg.transform, T_map_from_drone);

            // Look up transform from drone to camera's physical frame
            T_drone_from_physical_msg = _tf_buffer->lookupTransform(
                _drone_frame_id, _camera_frame_id, timestamp, timeout);
            tf2::fromMsg(T_drone_from_physical_msg.transform, T_drone_from_physical);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform chain from 'map' -> '%s' -> '%s': %s",
                _drone_frame_id.c_str(), _camera_frame_id.c_str(), ex.what());
            RCLCPP_WARN(this->get_logger(), "Could not get transform chain from 'map' -> '%s' -> '%s': %s",
                _drone_frame_id.c_str(), _camera_frame_id.c_str(), ex.what());
            return;
        }
        // Combine transforms to get the full transform from map to the camera's physical frame.
        tf2::Transform T_map_from_physical = T_map_from_drone * T_drone_from_physical;

        vision_msgs::msg::Detection3DArray detections_msg;
        detections_msg.header.stamp = msg->header.stamp;
        detections_msg.header.frame_id = "map"; // Final coordinates will be in the map frame.

        for (size_t i = 0; i < ids.size(); i++) {
            // Step 2: Estimate the marker's pose in the camera's optical frame using solvePnP.
            float half_size = _param_marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
            cv::Vec3d rvec, tvec;
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            
            // Step 2-1: Store the solvePnP result (marker frame -> camera optical frame transform)
            tf2::Transform T_optical_from_marker;
            T_optical_from_marker.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));
            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            T_optical_from_marker.getBasis().setValue(
                R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
            );
            
            // Step 2-2: Store marker-to-camera transform (마커 기준 드론 위치)
            _T_optical_from_marker_map[ids[i]] = T_optical_from_marker;
            
            // Step 2-3: Calculate and store camera-to-marker transform (드론 기준 마커 위치)
            tf2::Transform T_marker_from_optical = T_optical_from_marker.inverse();
            _T_marker_from_optical_map[ids[i]] = T_marker_from_optical;
            
            // Debug logging for stored transforms
            tf2::Vector3 marker_pos_from_camera = T_marker_from_optical.getOrigin();
            tf2::Vector3 camera_pos_from_marker = T_optical_from_marker.getOrigin();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Marker %d - Camera from marker: [%.3f, %.3f, %.3f], Marker from camera: [%.3f, %.3f, %.3f]",
                ids[i],
                camera_pos_from_marker.x(), camera_pos_from_marker.y(), camera_pos_from_marker.z(),
                marker_pos_from_camera.x(), marker_pos_from_camera.y(), marker_pos_from_camera.z());
            
            // Step 3: Combine all transforms to get the marker's pose in the world frame using stored transforms
            // T_map_from_marker = T_map_from_physical * T_physical_from_optical * T_optical_from_marker
            tf2::Transform T_map_from_marker = T_map_from_physical * _T_physical_from_optical * _T_optical_from_marker_map[ids[i]];
            
            // Step 3-1: Store the marker position in map frame
            _T_map_from_marker_map[ids[i]] = T_map_from_marker;
            
            tf2::Vector3 marker_pos_in_map = T_map_from_marker.getOrigin();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Marker %d calculated in map: X=%.2f, Y=%.2f, Z=%.2f",
                ids[i], marker_pos_in_map.x(), marker_pos_in_map.y(), marker_pos_in_map.z());
            
            // Additional logging for all stored coordinate systems
            camera_pos_from_marker = _T_optical_from_marker_map[ids[i]].getOrigin();
            marker_pos_from_camera = _T_marker_from_optical_map[ids[i]].getOrigin();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Marker %d stored coordinates - Map→Marker: [%.3f,%.3f,%.3f], Camera→Marker: [%.3f,%.3f,%.3f], Marker→Camera: [%.3f,%.3f,%.3f]",
                ids[i],
                marker_pos_in_map.x(), marker_pos_in_map.y(), marker_pos_in_map.z(),
                marker_pos_from_camera.x(), marker_pos_from_camera.y(), marker_pos_from_camera.z(),
                camera_pos_from_marker.x(), camera_pos_from_marker.y(), camera_pos_from_marker.z());
            
            // Step 4: Fill the message for publishing.
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
    
    // Draw markers and axes for visualization on the image topic
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
    if (!ids.empty()) {
        for (size_t i = 0; i < ids.size(); ++i) {
            cv::Vec3d rvec, tvec;
            float half_size = _param_marker_size / 2.0f;
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
            cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
            cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvec, tvec, _param_marker_size);
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
