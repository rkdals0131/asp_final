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
// vision_msgs 관련 헤더들은 hpp 파일에 모두 포함되어 있으므로 cpp 파일에서는 생략할 수 있습니다.
// 하지만 명확성을 위해 hpp 파일의 수정이 중요합니다.

MultiTrackerNode::MultiTrackerNode()
    : Node("multi_tracker_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting MultiTrackerNode");

    loadParameters();

    RCLCPP_INFO(this->get_logger(), "Loaded vehicle_type: %s", _vehicle_type.c_str());

    auto image_qos = rclcpp::QoS(1).best_effort();
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    _detectorParams = cv::aruco::DetectorParameters::create();
    _dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

    // TF 리스너 초기화
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
    // Publisher를 vision_msgs 형식으로 변경
    _marker_detections_pub = this->create_publisher<vision_msgs::msg::Detection3DArray>(_marker_detections_topic, pose_qos);
}

void MultiTrackerNode::loadParameters()
{
    declare_parameter<std::string>("vehicle_type", "x500");
    declare_parameter<int>("aruco_id", 0);
    declare_parameter<int>("dictionary", 0);
    declare_parameter<double>("marker_size", 0.5);
    declare_parameter<std::string>("vehicle_frame_id", "x500_gimbal_0/base_link"); // 드론 base_link frame ID

    get_parameter("vehicle_type", _vehicle_type);
    get_parameter("aruco_id", _param_aruco_id);
    get_parameter("dictionary", _param_dictionary);
    get_parameter("marker_size", _param_marker_size);
    get_parameter("vehicle_frame_id", _vehicle_frame_id);

    declare_parameter<std::string>("image_topic", "");
    declare_parameter<std::string>("camera_info_topic", "");
    declare_parameter<std::string>("target_id_topic", "/target_id");
    declare_parameter<std::string>("image_proc_topic", "/image_proc");
    // Topic 이름 변경
    declare_parameter<std::string>("marker_detections_topic", "/marker_detections");

    get_parameter("image_topic", _image_topic);
    get_parameter("camera_info_topic", _camera_info_topic);
    get_parameter("target_id_topic", _target_id_topic);
    get_parameter("image_proc_topic", _image_proc_topic);
    get_parameter("marker_detections_topic", _marker_detections_topic);
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
        // 여기서는 첫 번째로 감지된 마커만 사용합니다.
        size_t i = 0; 
        
        std_msgs::msg::Int32 id_msg;
        id_msg.data = ids[i];
        _target_id_pub->publish(id_msg);

        // objectPoints가 비어있으면 초기화
        if (objectPoints.empty()) {
            float half_size = _param_marker_size / 2.0f;
            objectPoints = {
                cv::Point3f(-half_size,  half_size, 0), cv::Point3f( half_size,  half_size, 0),
                cv::Point3f( half_size, -half_size, 0), cv::Point3f(-half_size, -half_size, 0)
            };
        }

        cv::Vec3d rvec, tvec;
        cv::solvePnP(objectPoints, corners[i], _camera_matrix, _dist_coeffs, rvec, tvec);

        cv::Mat rot_mat;
        cv::Rodrigues(rvec, rot_mat);
        cv::Mat rot_mat_inv = rot_mat.t();
        cv::Mat tvec_inv = -rot_mat_inv * cv::Mat(tvec);

        cv::Mat cv_rdf_to_rfu = (cv::Mat_<double>(3,3) <<
             0, -1,  0,
             1,  0,  0,
             0,  0,  1);
        
        cv::Mat rot_mat_rfu = cv_rdf_to_rfu * rot_mat_inv;
        cv::Mat tvec_rfu_mat = cv_rdf_to_rfu * tvec_inv;

        tf2::Matrix3x3 tf_rot_marker_to_cam_rfu(
            rot_mat_rfu.at<double>(0,0), rot_mat_rfu.at<double>(0,1), rot_mat_rfu.at<double>(0,2),
            rot_mat_rfu.at<double>(1,0), rot_mat_rfu.at<double>(1,1), rot_mat_rfu.at<double>(1,2),
            rot_mat_rfu.at<double>(2,0), rot_mat_rfu.at<double>(2,1), rot_mat_rfu.at<double>(2,2)
        );
        tf2::Transform T_marker_to_camera_rfu;
        T_marker_to_camera_rfu.setBasis(tf_rot_marker_to_cam_rfu);
        T_marker_to_camera_rfu.setOrigin(tf2::Vector3(tvec_rfu_mat.at<double>(0), tvec_rfu_mat.at<double>(1), tvec_rfu_mat.at<double>(2)));
        
        tf2::Transform T_vehicle_to_camera;
        T_vehicle_to_camera.setOrigin(tf2::Vector3(0.0, 0.0, -0.1));
        T_vehicle_to_camera.setRotation(tf2::Quaternion(0, 0, 0, 1));

        tf2::Transform T_marker_to_vehicle = T_marker_to_camera_rfu * T_vehicle_to_camera.inverse();
        
        geometry_msgs::msg::TransformStamped T_map_to_vehicle_msg;
        try {
            T_map_to_vehicle_msg = _tf_buffer->lookupTransform("map", _vehicle_frame_id, rclcpp::Time(0));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to 'map': %s", _vehicle_frame_id.c_str(), ex.what());
            return;
        }
        tf2::Transform T_map_to_vehicle;
        tf2::fromMsg(T_map_to_vehicle_msg.transform, T_map_to_vehicle);

        tf2::Transform T_vehicle_to_marker = T_marker_to_vehicle.inverse();
        tf2::Transform T_map_to_marker = T_map_to_vehicle * T_vehicle_to_marker;

        vision_msgs::msg::Detection3DArray detections_msg;
        detections_msg.header.stamp = msg->header.stamp;
        detections_msg.header.frame_id = "map";

        vision_msgs::msg::Detection3D detection;
        detection.header = detections_msg.header;
        
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = std::to_string(ids[i]);
        hypothesis.hypothesis.score = 1.0;
        tf2::toMsg(T_map_to_marker, hypothesis.pose.pose);
        
        detection.results.push_back(hypothesis);
        detections_msg.detections.push_back(detection);
        
        _marker_detections_pub->publish(detections_msg);

        _target[0] = hypothesis.pose.pose.position.x;
        _target[1] = hypothesis.pose.pose.position.y;
        _target[2] = hypothesis.pose.pose.position.z;
    }

    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
    if (!ids.empty() && !_camera_matrix.empty() && !_dist_coeffs.empty()) {
        cv::Vec3d rvec, tvec;
        cv::solvePnP(objectPoints, corners[0], _camera_matrix, _dist_coeffs, rvec, tvec);
        cv::drawFrameAxes(cv_ptr->image, _camera_matrix, _dist_coeffs, rvec, tvec, _param_marker_size);
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
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    if(detected){
        stream << "World X:" << _target[0] << " Y:" << _target[1] << " Z:" << _target[2];
    } else {
        stream << "No marker detected";
    }
    std::string text_xyz = stream.str();
    cv::putText(image->image, text_xyz, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
