#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomTfBridge : public rclcpp::Node
{
public:
  OdomTfBridge()
  : Node("odom_tf_bridge")
  {
    // Parameters
    this->declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Republish odometry with correct frame IDs (optional but useful)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // QoS: Match the Unitree publisher which uses RELIABLE + VOLATILE
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&OdomTfBridge::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "OdomTfBridge initialized:");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing TF: %s -> %s", 
                odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Republishing odom to: /odom");
    RCLCPP_INFO(this->get_logger(), "  QoS: RELIABLE + VOLATILE");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    msg_count_++;
    
    // Log first message received
    if (msg_count_ == 1) {
      RCLCPP_INFO(this->get_logger(), "First odometry message received from %s", 
                  odom_topic_.c_str());
    }
    
    // Log periodically
    if (msg_count_ % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), "Received %lu odometry messages", msg_count_);
    }

    // Publish TF
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      // Use current ROS time for consistency with other nodes
      // (Unitree robot clock may differ from host clock)
      t.header.stamp = this->now();
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;

      t.transform.translation.x = msg->pose.pose.position.x;
      t.transform.translation.y = msg->pose.pose.position.y;
      t.transform.translation.z = msg->pose.pose.position.z;
      t.transform.rotation = msg->pose.pose.orientation;

      tf_broadcaster_->sendTransform(t);
    }

    // Republish odometry with our frame IDs
    auto odom_out = *msg;
    odom_out.header.stamp = this->now();  // Fix timestamp to match host clock
    odom_out.header.frame_id = odom_frame_;
    odom_out.child_frame_id = base_frame_;
    odom_pub_->publish(odom_out);
  }

  // Parameters
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};

  // State
  size_t msg_count_{0};

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTfBridge>();
  
  RCLCPP_INFO(node->get_logger(), "OdomTfBridge node starting...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}