/**
 * @file test_movement_node.cpp
 * @brief Simple test node to send velocity commands to the Go2
 * 
 * Usage:
 *   ros2 run go2_navigation test_movement --ros-args -p command:=forward
 *   ros2 run go2_navigation test_movement --ros-args -p command:=backward
 *   ros2 run go2_navigation test_movement --ros-args -p command:=left
 *   ros2 run go2_navigation test_movement --ros-args -p command:=right
 *   ros2 run go2_navigation test_movement --ros-args -p command:=spin
 *   ros2 run go2_navigation test_movement --ros-args -p command:=stop
 * 
 * Optional parameters:
 *   -p duration:=3.0        Duration in seconds (default: 2.0)
 *   -p speed:=0.5           Speed multiplier (default: 1.0)
 *   -p cmd_vel_topic:=/cmd_vel  Topic to publish to
 */

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TestMovementNode : public rclcpp::Node
{
public:
  TestMovementNode()
  : Node("test_movement")
  {
    // Parameters
    this->declare_parameter<std::string>("command", "stop");
    this->declare_parameter<double>("duration", 2.0);
    this->declare_parameter<double>("speed", 1.0);
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    command_ = this->get_parameter("command").as_string();
    duration_ = this->get_parameter("duration").as_double();
    speed_multiplier_ = this->get_parameter("speed").as_double();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    // Set velocities based on command
    set_velocities_from_command();

    // Publisher
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // Timer for publishing
    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TestMovementNode::timer_callback, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "=================================");
    RCLCPP_INFO(this->get_logger(), "Test Movement Node");
    RCLCPP_INFO(this->get_logger(), "=================================");
    RCLCPP_INFO(this->get_logger(), "Command: %s", command_.c_str());
    RCLCPP_INFO(this->get_logger(), "Duration: %.1f seconds", duration_);
    RCLCPP_INFO(this->get_logger(), "Velocity: vx=%.2f, vy=%.2f, wz=%.2f", vx_, vy_, wz_);
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "=================================");
  }

private:
  void set_velocities_from_command()
  {
    // Base velocities (conservative)
    const double base_linear = 0.3;   // m/s
    const double base_angular = 0.5;  // rad/s

    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;

    if (command_ == "forward") {
      vx_ = base_linear * speed_multiplier_;
    } else if (command_ == "backward") {
      vx_ = -base_linear * speed_multiplier_;
    } else if (command_ == "left") {
      vy_ = base_linear * speed_multiplier_;
    } else if (command_ == "right") {
      vy_ = -base_linear * speed_multiplier_;
    } else if (command_ == "spin" || command_ == "spin_left") {
      wz_ = base_angular * speed_multiplier_;
    } else if (command_ == "spin_right") {
      wz_ = -base_angular * speed_multiplier_;
    } else if (command_ == "stop") {
      // All zeros
    } else if (command_ == "diagonal_fl") {
      // Forward-left diagonal
      vx_ = base_linear * speed_multiplier_ * 0.707;
      vy_ = base_linear * speed_multiplier_ * 0.707;
    } else if (command_ == "diagonal_fr") {
      // Forward-right diagonal
      vx_ = base_linear * speed_multiplier_ * 0.707;
      vy_ = -base_linear * speed_multiplier_ * 0.707;
    } else if (command_ == "circle") {
      // Move in a circle (forward + rotation)
      vx_ = base_linear * speed_multiplier_ * 0.5;
      wz_ = base_angular * speed_multiplier_;
    } else {
      RCLCPP_WARN(this->get_logger(), 
        "Unknown command '%s'. Valid commands: forward, backward, left, right, "
        "spin, spin_left, spin_right, stop, diagonal_fl, diagonal_fr, circle",
        command_.c_str());
    }
  }

  void timer_callback()
  {
    auto elapsed = (this->now() - start_time_).seconds();

    geometry_msgs::msg::Twist msg;

    if (elapsed < duration_) {
      // Send commanded velocity
      msg.linear.x = vx_;
      msg.linear.y = vy_;
      msg.angular.z = wz_;

      // Progress indicator every 0.5 seconds
      if (static_cast<int>(elapsed * 2) > last_progress_) {
        last_progress_ = static_cast<int>(elapsed * 2);
        RCLCPP_INFO(this->get_logger(), "Moving... %.1f / %.1f seconds", 
                    elapsed, duration_);
      }
    } else if (!finished_) {
      // Send stop and shutdown
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(), "Movement complete. Sending stop command.");
      
      // Publish stop a few times to ensure it's received
      for (int i = 0; i < 10; ++i) {
        pub_->publish(msg);
        std::this_thread::sleep_for(20ms);
      }

      finished_ = true;
      RCLCPP_INFO(this->get_logger(), "Done. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    pub_->publish(msg);
  }

  // Parameters
  std::string command_;
  double duration_{2.0};
  double speed_multiplier_{1.0};
  std::string cmd_vel_topic_;
  double publish_rate_hz_{50.0};

  // Computed velocities
  double vx_{0.0};
  double vy_{0.0};
  double wz_{0.0};

  // State
  rclcpp::Time start_time_;
  int last_progress_{-1};
  bool finished_{false};

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TestMovementNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}