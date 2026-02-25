/**
 * @file cmdvel_to_sport_bridge.cpp
 * @brief Bridges geometry_msgs/TwistStamped commands to Unitree Go2 Sport API
 * 
 * This node subscribes to /cmd_vel and translates velocity commands
 * to Unitree's sport API format, publishing to /api/sport/request.
 */

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class CmdVelToSportBridge : public rclcpp::Node
{
public:
  CmdVelToSportBridge()
  : Node("cmdvel_to_sport_bridge"),
    sport_client_(this)
  {
    // Parameters
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<double>("cmd_timeout_sec", 0.5);
    this->declare_parameter<bool>("send_standup_on_start", true);
    this->declare_parameter<double>("max_linear_vel", 1.0);
    this->declare_parameter<double>("max_angular_vel", 1.5);
    this->declare_parameter<bool>("use_twist_stamped", true);  // Nav2 uses TwistStamped

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    cmd_timeout_sec_ = this->get_parameter("cmd_timeout_sec").as_double();
    send_standup_on_start_ = this->get_parameter("send_standup_on_start").as_bool();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    use_twist_stamped_ = this->get_parameter("use_twist_stamped").as_bool();

    // Publisher for sport API requests
    req_pub_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", 10);

    // Subscribe to cmd_vel - support both Twist and TwistStamped
    if (use_twist_stamped_) {
      sub_stamped_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, rclcpp::QoS(10),
        std::bind(&CmdVelToSportBridge::cmd_vel_stamped_callback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribing to TwistStamped on %s", cmd_vel_topic_.c_str());
    } else {
      sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10),
        std::bind(&CmdVelToSportBridge::cmd_vel_callback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribing to Twist on %s", cmd_vel_topic_.c_str());
    }

    // Optional: Subscribe to sport mode state for feedback
    state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/sportmodestate", rclcpp::QoS(10),
      std::bind(&CmdVelToSportBridge::state_callback, this, std::placeholders::_1));

    // Timer to continuously send commands
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelToSportBridge::timer_callback, this));

    last_cmd_time_ = this->now();

    RCLCPP_INFO(get_logger(), "CmdVel to Sport Bridge initialized");
    RCLCPP_INFO(get_logger(), "  Publishing to: /api/sport/request");
    RCLCPP_INFO(get_logger(), "  Rate: %.1f Hz, Timeout: %.2f s", 
                publish_rate_hz_, cmd_timeout_sec_);
    RCLCPP_INFO(get_logger(), "  Max velocities: linear=%.2f m/s, angular=%.2f rad/s",
                max_linear_vel_, max_angular_vel_);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = this->now();
    cmd_received_ = true;
    RCLCPP_DEBUG(get_logger(), "Twist received: vx=%.2f, vy=%.2f, wz=%.2f", 
                 msg->linear.x, msg->linear.y, msg->angular.z);
  }

  void cmd_vel_stamped_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_cmd_ = msg->twist;
    last_cmd_time_ = this->now();
    cmd_received_ = true;
    RCLCPP_DEBUG(get_logger(), "TwistStamped received: vx=%.2f, vy=%.2f, wz=%.2f", 
                 msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
  }

  void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    current_mode_ = msg->mode;
  }

  void timer_callback()
  {
    unitree_api::msg::Request req;

    // Handle initial standup
    if (send_standup_on_start_ && !did_standup_) {
      RCLCPP_INFO(get_logger(), "Sending StandUp command...");
      sport_client_.StandUp(req);
      req_pub_->publish(req);
      did_standup_ = true;
      standup_time_ = this->now();
      return;
    }

    // Wait a bit after standup before accepting move commands
    if (did_standup_ && !ready_to_move_) {
      auto elapsed = (this->now() - standup_time_).seconds();
      if (elapsed > 2.0) {
        ready_to_move_ = true;
        RCLCPP_INFO(get_logger(), "Ready to receive movement commands");
      } else {
        sport_client_.StandUp(req);
        req_pub_->publish(req);
        return;
      }
    }

    // Check for command timeout
    const auto age = (this->now() - last_cmd_time_).seconds();
    const bool timed_out = !cmd_received_ || (age > cmd_timeout_sec_);

    // Get velocities (zero if timed out)
    double vx = timed_out ? 0.0 : last_cmd_.linear.x;
    double vy = timed_out ? 0.0 : last_cmd_.linear.y;
    double wz = timed_out ? 0.0 : last_cmd_.angular.z;

    // Clamp velocities
    vx = std::clamp(vx, -max_linear_vel_, max_linear_vel_);
    vy = std::clamp(vy, -max_linear_vel_, max_linear_vel_);
    wz = std::clamp(wz, -max_angular_vel_, max_angular_vel_);

    // Check if velocities are effectively zero
    const double eps = 1e-3;
    const bool is_zero = (std::abs(vx) < eps && std::abs(vy) < eps && std::abs(wz) < eps);

    if (is_zero) {
      sport_client_.StopMove(req);
      if (was_moving_) {
        RCLCPP_INFO(get_logger(), "Stopping movement");
        was_moving_ = false;
      }
    } else {
      sport_client_.Move(req, 
                          static_cast<float>(vx), 
                          static_cast<float>(vy), 
                          static_cast<float>(wz));
      if (!was_moving_) {
        RCLCPP_INFO(get_logger(), "Starting movement: vx=%.2f, vy=%.2f, wz=%.2f", 
                     vx, vy, wz);
        was_moving_ = true;
      }
    }

    req_pub_->publish(req);
  }

  // SportClient
  SportClient sport_client_;

  // Parameters
  std::string cmd_vel_topic_;
  double publish_rate_hz_{50.0};
  double cmd_timeout_sec_{0.5};
  bool send_standup_on_start_{true};
  double max_linear_vel_{1.0};
  double max_angular_vel_{1.5};
  bool use_twist_stamped_{true};

  // State
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time standup_time_;
  bool cmd_received_{false};
  bool did_standup_{false};
  bool ready_to_move_{false};
  bool was_moving_{false};
  int current_mode_{0};

  // ROS interfaces
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_stamped_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToSportBridge>();
  RCLCPP_INFO(node->get_logger(), "CmdVel to Sport Bridge node starting...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}