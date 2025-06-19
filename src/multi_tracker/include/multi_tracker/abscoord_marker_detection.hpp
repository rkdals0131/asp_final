// multi_tracker_node.hpp
#ifndef MULTI_TRACKER_NODE_HPP_
#define MULTI_TRACKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"

// vision_msgs
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Transform.h>
#include <map>


class MultiTrackerNode : public rclcpp::Node
{
public:
    MultiTrackerNode();

    // Public accessors for stored marker coordinates
    const std::map<int, tf2::Transform>& getMarkerFromCameraRdfTransforms() const { return _T_marker_from_camera_rdf_map; }
    const std::map<int, tf2::Transform>& getCameraRdfFromMarkerTransforms() const { return _T_camera_rdf_from_marker_map; }
    const std::map<int, tf2::Transform>& getMapFromMarkerTransforms() const { return _T_map_from_marker_map; }

private:
    void loadParameters();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // Command callback for landing mode control
    void command_callback(const std_msgs::msg::String::SharedPtr msg);

    // ROS 2 members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _command_sub;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _marker_detections_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;

    // TF2 members
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

    // Camera parameters
    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;

    // ArUco detection
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    std::string _image_topic, _camera_info_topic, _camera_frame_id, _image_proc_topic;
    std::string _marker_detections_topic;
    int _param_dictionary;
    double _param_marker_size;
    double _param_landing_marker_size;

    // Landing mode control
    bool _is_landing_mode = false;

    // Marker position storage
    std::map<int, tf2::Transform> _T_camera_rdf_from_marker_map;  // 마커 기준 카메라 RDF 위치
    std::map<int, tf2::Transform> _T_marker_from_camera_rdf_map;  // 카메라 RDF 기준 마커 위치
    std::map<int, tf2::Transform> _T_map_from_marker_map;         // 맵 기준 마커 위치
};

#endif // MULTI_TRACKER_NODE_HPP_
