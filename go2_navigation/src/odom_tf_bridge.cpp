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

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&OdomTfBridge::odom_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Publishing TF %s -> %s from %s",
      odom_frame_.c_str(), base_frame_.c_str(), odom_topic_.c_str());
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBridge>());
  rclcpp::shutdown();
  return 0;
}
