// multi_tracker_node.hpp
#ifndef MULTI_TRACKER_NODE_HPP_
#define MULTI_TRACKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/int32.hpp"

// vision_msgs 관련 헤더 추가
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class MultiTrackerNode : public rclcpp::Node
{
public:
    MultiTrackerNode();

private:
    void loadParameters();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void annotate_image(cv_bridge::CvImagePtr image, bool detected);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
    
    // 이전 Publisher들
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _target_id_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
    
    // 새로운 Detection3DArray Publisher
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _marker_detections_pub;

    // TF 리스너 및 버퍼
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;

    // 파라미터 변수
    std::string _camera_frame_id;
    int _param_dictionary;
    double _param_marker_size;

    std::string _optical_frame_id;
    std::string _image_topic;
    std::string _camera_info_topic;
    std::string _target_id_topic;
    std::string _image_proc_topic;
    std::string _marker_detections_topic;

    double _target[3] = {0.0, 0.0, 0.0};
};

#endif // MULTI_TRACKER_NODE_HPP_
