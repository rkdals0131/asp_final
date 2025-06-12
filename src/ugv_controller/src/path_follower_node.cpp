#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using std::placeholders::_1;

class PID {
public:
  PID(double Kp, double Ki, double Kd)
  : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = dt > 0.0 ? (error - prev_error_) / dt : 0.0;
    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = error;
    return output;
  }

private:
  double Kp_, Ki_, Kd_;
  double prev_error_, integral_;
};

class PathFollower : public rclcpp::Node {
public:
  PathFollower()
  : Node("path_follower"),
    origin_set_(false), pose_received_(false), navsat_received_(false),
    current_target_index_(0),
    pid_speed_(1.0, 0.0, 0.1), pid_steer_(2.0, 0.0, 0.1)
  {
    declare_parameter<std::string>("path_file", "path.csv");
    declare_parameter<double>("longitudinal_speed", 1.0);
    get_parameter("path_file", path_file_);
    get_parameter("longitudinal_speed", max_speed_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
    navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/world/default/model/X1_asp/link/base_link/sensor/navsat_sensor/navsat", 10,
      std::bind(&PathFollower::navsat_callback, this, _1));
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/model/X1_asp/pose", 10,
      std::bind(&PathFollower::pose_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/model/X1/odometry",          // Gazebo DiffDrive 기본 오도메토리 토픽
    rclcpp::QoS(10),
    std::bind(&PathFollower::odometry_callback, this, _1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);


    load_path();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathFollower::control_loop, this));
  }

private:
    void load_path() 
    {
        std::ifstream file(path_file_);
        RCLCPP_INFO(get_logger(), "Loading path from: %s", path_file_.c_str());
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", path_file_.c_str());
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str)) {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            path_.emplace_back(x, y);
            }
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu path points.", path_.size());
    }


  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!origin_set_) {
      origin_lat_ = msg->latitude;
      origin_lon_ = msg->longitude;
      origin_alt_ = msg->altitude;
      origin_set_ = true;
      RCLCPP_INFO(get_logger(), "Origin set: [%.6f, %.6f]", origin_lat_, origin_lon_);
    }

    // Naive ECEF-like conversion for demo (not accurate like GeographicLib)
    constexpr double earth_radius = 6371000.0; // meters
    double lat_rad = msg->latitude * M_PI / 180.0;
    double lon_rad = msg->longitude * M_PI / 180.0;
    double dlat = (msg->latitude - origin_lat_) * M_PI / 180.0;
    double dlon = (msg->longitude - origin_lon_) * M_PI / 180.0;
    current_x_ = earth_radius * dlon * cos(lat_rad);
    current_y_ = earth_radius * dlat;
    current_z_ = msg->altitude - origin_alt_;

    navsat_received_ = true;
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    pose_received_ = true;
  }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // ─ ① 위치 갱신
        // current_x_ = msg->pose.pose.position.x;
        // current_y_ = msg->pose.pose.position.y;
        // current_z_ = msg->pose.pose.position.z;

        // ─ ② 자세(yaw) 갱신
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);   // 라디안
        current_yaw_ = yaw;

        pose_received_ = true;
    }

  void control_loop() {
    if (!origin_set_ || !navsat_received_ || !pose_received_ || path_.empty()) 
    {
        if(!origin_set_) {
            RCLCPP_WARN(get_logger(), "Origin not set yet.");
        }
        if(!navsat_received_) {
            RCLCPP_WARN(get_logger(), "NavSat data not received yet.");
        }
        if(!pose_received_) {
            RCLCPP_WARN(get_logger(), "Pose data not received yet.");
        }
        if(path_.empty()) {
            RCLCPP_WARN(get_logger(), "Path not loaded yet.");
        }
        return;
    }
    if (!path_localized_ && origin_set_ && pose_received_ && !path_.empty()) {
        for (auto &[px, py] : path_) 
        {
            double gx = current_x_ + std::cos(current_yaw_)*px - std::sin(current_yaw_)*py;
            double gy = current_y_ + std::sin(current_yaw_)*px + std::cos(current_yaw_)*py;
            path_world_.emplace_back(gx, gy);
        }
        path_localized_ = true;
        RCLCPP_INFO(get_logger(), "Body-frame path converted → world frame (%zu pts)", path_world_.size());
        // 초기 목표 인덱스 그대로 0
    }

    if (!path_localized_) return;   // 아직 변환 전이면 대기

    // 현재 추종할 월드-목표
    double goal_x = path_world_[current_target_index_].first;
    double goal_y = path_world_[current_target_index_].second;

    // 오차·각도
    double ex   = goal_x - current_x_;
    double ey   = goal_y - current_y_;
    double dist = std::hypot(ex, ey);
    double target_ang = std::atan2(ey, ex);
    double ang_err = std::atan2(std::sin(target_ang - current_yaw_),
                                std::cos(target_ang - current_yaw_)); // −π~π

    // double ang_err = std::atan2(std::sin(target_ang - current_yaw_), std::cos(target_ang - current_yaw_));
    RCLCPP_INFO(this->get_logger(), "dist=%.2f, target_ang=%.2f, current_yaw=%.2f, ang_err=%.2f", dist, target_ang, current_yaw_, ang_err);

    // PID
    double dt = 0.1;
    double speed_cmd = pid_speed_.compute(dist, dt);
    double steer_cmd = pid_steer_.compute(ang_err, dt);
    speed_cmd = std::clamp(speed_cmd, -max_speed_, max_speed_);


    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed_cmd;
    cmd.angular.z = steer_cmd;
    RCLCPP_INFO(get_logger(), "Speed: %.2f, Steering: %.2f", speed_cmd, steer_cmd);
    cmd_pub_->publish(cmd);

    // 목표 도달 판정(0.5 m) → 다음 포인트
    if (dist < 0.5 && current_target_index_+1 < path_world_.size())
        ++current_target_index_;

    // [1] 현재 위치 퍼블리시 (current_pose)
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = "map";  // RViz에서 사용할 frame
    pose_msg.pose.position.x = current_x_;
    pose_msg.pose.position.y = current_y_;
    pose_msg.pose.position.z = current_z_;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_);
    pose_msg.pose.orientation = tf2::toMsg(q);
    pose_pub_->publish(pose_msg);

    // [2] 목표 마커 퍼블리시 (target_marker)
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
    marker.pose.position.z = 0.2;  // 살짝 띄움
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_pub_->publish(marker);

  }

    std::string path_file_;
    double max_speed_;
    bool origin_set_, navsat_received_, pose_received_;
    double origin_lat_, origin_lon_, origin_alt_;
    double current_x_{0.0}, current_y_{0.0}, current_z_{0.0}, current_yaw_{0.0};
    std::vector<std::pair<double,double>> path_world_;
    bool path_localized_{false};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double,double>> path_;
    size_t current_target_index_;
    PID pid_speed_, pid_steer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}