#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <string>
#include <memory>

class PoseTFBroadcaster : public rclcpp::Node
{
public:
  PoseTFBroadcaster()
  : Node("pose_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 세 개의 pose_static 토픽 구독
    sub_x1_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X1_asp/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose_static", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));

    sub_x500_dynamic_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/x500_gimbal_0/pose", 10,
      std::bind(&PoseTFBroadcaster::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (auto transform : msg->transforms)
    {
      // 1. frame_id가 "default"인 경우 "map"으로 변경
      if (transform.header.frame_id == "default") {
        transform.header.frame_id = "map";
      }

      // 2. 부모/자식 프레임 처리 (A/B -> B)
    //   std::string full_child = transform.child_frame_id;
    //   size_t slash_pos = full_child.rfind('/');
    //   if (slash_pos != std::string::npos) {
    //     transform.child_frame_id = full_child.substr(slash_pos + 1);
    //   }

      // 3. TF 퍼블리시
      tf_broadcaster_->sendTransform(transform);

      // 4. x500_gimbal_0/camera_link에 대해 RDF 변환된 프레임 추가 생성
      if (transform.child_frame_id == "x500_gimbal_0/camera_link")
      {
        create_rdf_camera_transform(transform);
      }
    }
  }

  void create_rdf_camera_transform(const geometry_msgs::msg::TransformStamped& original_transform)
  {
    geometry_msgs::msg::TransformStamped rdf_transform = original_transform;
    
    // 새로운 child_frame_id 설정
    rdf_transform.child_frame_id = "x500_gimbal_0/camera_link_rdf";
    
    // 원래 회전을 쿼터니언으로 받아오기
    tf2::Quaternion original_quat(
      original_transform.transform.rotation.x,
      original_transform.transform.rotation.y,
      original_transform.transform.rotation.z,
      original_transform.transform.rotation.w
    );
    
    // BRU -> RDF 변환 행렬을 쿼터니언으로 변환
    // rotation_matrix_rdf = [
    // [0, 0, -1], 
    // [1, 0, 0], 
    // [0, -1, 0]
    // ]
    // 이는 Y축으로 90도, X축으로 -90도 회전과 동일
    tf2::Quaternion rdf_rotation;
    rdf_rotation.setRPY(-M_PI/2, 0, M_PI/2);  // RDF 변환
    
    // 원래 회전과 RDF 변환 합성
    tf2::Quaternion final_quat = original_quat * rdf_rotation;
    
    // 새로운 변환에 적용
    rdf_transform.transform.rotation.x = final_quat.x();
    rdf_transform.transform.rotation.y = final_quat.y();
    rdf_transform.transform.rotation.z = final_quat.z();
    rdf_transform.transform.rotation.w = final_quat.w();
    
    // Translation은 그대로 유지 (원점은 동일)
    
    // RDF 변환된 TF 퍼블리시
    tf_broadcaster_->sendTransform(rdf_transform);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x1_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_static_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_x500_dynamic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}