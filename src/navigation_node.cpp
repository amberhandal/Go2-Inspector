/**
 * @file navigation_node.cpp
 * @brief Simple navigation node that publishes velocity commands
 * 
 * This node provides /walk and /stop services for basic movement control.
 * It publishes geometry_msgs/Twist to /cmd_vel which is then bridged
 * to the Unitree Sport API by cmdvel_to_sport_bridge.
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("navigation_node")
  {
    // Parameters
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    this->declare_parameter<double>("vx", 0.2);
    this->declare_parameter<double>("vy", 0.0);
    this->declare_parameter<double>("wz", 0.0);

    this->declare_parameter<bool>("auto_start", false);
    this->declare_parameter<double>("duration_s", 1.0);

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    vx_ = this->get_parameter("vx").as_double();
    vy_ = this->get_parameter("vy").as_double();
    wz_ = this->get_parameter("wz").as_double();

    auto_start_ = this->get_parameter("auto_start").as_bool();
    duration_s_ = this->get_parameter("duration_s").as_double();

    // Publisher
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // Timer to publish continuously
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NavigationNode::on_timer, this));

    // Services
    walk_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "walk",
      std::bind(&NavigationNode::on_walk, this, std::placeholders::_1, std::placeholders::_2));

    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "stop",
      std::bind(&NavigationNode::on_stop, this, std::placeholders::_1, std::placeholders::_2));

    // Auto-start behavior (optional)
    if (auto_start_) {
      start_walking_for(duration_s_);
    }

    RCLCPP_INFO(this->get_logger(),
      "NavigationNode publishing to %s at %.1f Hz. Services: /walk, /stop",
      cmd_vel_topic_.c_str(), publish_rate_hz_);
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::Twist msg;
    if (walking_) {
      msg.linear.x  = vx_;
      msg.linear.y  = vy_;
      msg.angular.z = wz_;

      // if we are in timed-walk mode, stop when time is up
      if (timed_stop_enabled_) {
        if (this->now() >= stop_time_) {
          walking_ = false;
          timed_stop_enabled_ = false;
          RCLCPP_INFO(this->get_logger(), "Timed walk finished -> stopping.");
        }
      }
    } else {
      // publish zeros to actively stop
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.angular.z = 0.0;
    }

    pub_->publish(msg);
  }

  void start_walking_for(double duration_s)
  {
    walking_ = true;
    timed_stop_enabled_ = true;
    stop_time_ = this->now() + rclcpp::Duration::from_seconds(duration_s);
    RCLCPP_INFO(this->get_logger(),
      "Walking for %.2f s (vx=%.3f vy=%.3f wz=%.3f) then stopping.",
      duration_s, vx_, vy_, wz_);
  }

  void on_walk(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // walk for duration_s_ whenever /walk is called
    start_walking_for(duration_s_);
    res->success = true;
    res->message = "Walking started (timed).";
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    walking_ = false;
    timed_stop_enabled_ = false;
    res->success = true;
    res->message = "Stopped.";
    RCLCPP_INFO(this->get_logger(), "Stop requested.");
  }

  std::string cmd_vel_topic_;
  double publish_rate_hz_{50.0};
  double vx_{0.2}, vy_{0.0}, wz_{0.0};
  bool auto_start_{false};
  double duration_s_{1.0};

  bool walking_{false};
  bool timed_stop_enabled_{false};
  rclcpp::Time stop_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr walk_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}