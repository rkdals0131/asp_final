#include "multi_tracker/multi_tracker_node.hpp"
#include <sstream>
#include <iomanip>

// TF2 관련 헤더
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Transform.h>

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

    // 기존 Publisher들
    _target_id_pub = this->create_publisher<std_msgs::msg::Int32>(_target_id_topic, pose_qos);
    _image_pub = this->create_publisher<sensor_msgs::msg::Image>(_image_proc_topic, image_qos);
    
    // 새로운 Publisher 생성
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
    declare_parameter<std::string>("marker_detections_topic", "/marker_detections"); // 새 토픽 이름

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
        RCLCPP_WARN(this->get_logger(), "Camera intrinsics are not initialized yet.");
        return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids, _detectorParams);
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    if (ids.empty()) {
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg().get());
        return;
    }

    geometry_msgs::msg::TransformStamped T_map_to_cam_msg;
    try {
        T_map_to_cam_msg = _tf_buffer->lookupTransform("map", _camera_frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to 'map': %s", _camera_frame_id.c_str(), ex.what());
        return;
    }
    tf2::Transform T_map_to_cam;
    tf2::fromMsg(T_map_to_cam_msg.transform, T_map_to_cam);

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
        cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);
        cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvec, tvec, _param_marker_size);

        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        tf2::Matrix3x3 R_cam_to_marker_tf(
            R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
            R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
            R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
        );
        tf2::Vector3 t_cam_to_marker_tf(tvec[0], tvec[1], tvec[2]);
        tf2::Transform T_cam_to_marker(R_cam_to_marker_tf, t_cam_to_marker_tf);
        tf2::Transform T_map_to_marker = T_map_to_cam * T_cam_to_marker;

        vision_msgs::msg::Detection3D detection;
        detection.header = detections_msg.header;

        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        
        // <<< FIX 1: 'id' -> 'class_id' 수정 >>>
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
        }
    }

    if (!detections_msg.detections.empty()) {
        _marker_detections_pub->publish(detections_msg);
    }

    annotate_image(cv_ptr);
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

void MultiTrackerNode::annotate_image(cv_bridge::CvImagePtr image)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "World X:"  << _target[0] << " Y:" << _target[1]  << " Z:" << _target[2];
    std::string text_xyz = stream.str();

    cv::Point textOrg(10, 30);
    // <<< FIX 2: putText 함수에 좌표(textOrg) 인자 추가 >>>
    cv::putText(image->image, text_xyz, textOrg, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
