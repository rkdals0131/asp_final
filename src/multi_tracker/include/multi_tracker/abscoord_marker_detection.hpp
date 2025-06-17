// multi_tracker_node.hpp
#ifndef MULTI_TRACKER_NODE_HPP_
#define MULTI_TRACKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

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
    const std::map<int, tf2::Transform>& getMarkerFromOpticalTransforms() const { return _T_marker_from_optical_map; }
    const std::map<int, tf2::Transform>& getOpticalFromMarkerTransforms() const { return _T_optical_from_marker_map; }
    const std::map<int, tf2::Transform>& getMapFromMarkerTransforms() const { return _T_map_from_marker_map; }

private:
    void loadParameters();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _marker_detections_pub;

    // TF
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
    tf2::Transform _T_physical_from_optical;

    // Marker position storage
    std::map<int, tf2::Transform> _T_optical_from_marker_map;  // 마커 기준 드론(카메라) 위치
    std::map<int, tf2::Transform> _T_marker_from_optical_map;  // 드론(카메라) 기준 마커 위치
    std::map<int, tf2::Transform> _T_map_from_marker_map;      // 맵 기준 마커 위치

    // OpenCV / Aruco
    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;

    // Parameters
    std::string _camera_frame_id;
    int _param_dictionary;
    double _param_marker_size;

    std::string _image_topic;
    std::string _camera_info_topic;
    std::string _image_proc_topic;
    std::string _marker_detections_topic;
    std::string _drone_frame_id;
};

#endif // MULTI_TRACKER_NODE_HPP_
