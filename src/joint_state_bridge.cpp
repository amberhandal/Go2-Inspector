/**
 * @file joint_state_bridge.cpp
 * @brief Converts Unitree Go2 low-level state to sensor_msgs/JointState
 * 
 * Subscribes to /lowstate (unitree_go/msg/LowState) and publishes
 * /joint_states (sensor_msgs/msg/JointState) for robot_state_publisher.
 */

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_go/msg/low_state.hpp"

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge()
  : Node("joint_state_bridge")
  {
    // Parameters
    this->declare_parameter<std::string>("lowstate_topic", "/lowstate");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");

    lowstate_topic_ = this->get_parameter("lowstate_topic").as_string();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();

    // Joint names matching the URDF
    // Go2 has 12 joints: 4 legs x 3 joints (hip, thigh, calf)
    // Order in Unitree lowstate: FR, FL, RR, RL (each: hip, thigh, calf)
    joint_names_ = {
      // Front Right
      "FR_hip_joint",
      "FR_thigh_joint", 
      "FR_calf_joint",
      // Front Left
      "FL_hip_joint",
      "FL_thigh_joint",
      "FL_calf_joint",
      // Rear Right
      "RR_hip_joint",
      "RR_thigh_joint",
      "RR_calf_joint",
      // Rear Left
      "RL_hip_joint",
      "RL_thigh_joint",
      "RL_calf_joint"
    };

    // Publisher using SensorDataQoS (standard for sensor data like joint_states)
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS());

    // Subscriber with QoS matching Unitree
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      lowstate_topic_, qos,
      std::bind(&JointStateBridge::lowstate_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JointStateBridge initialized:");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", lowstate_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", joint_states_topic_.c_str());
  }

private:
  void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
  {
    msg_count_++;

    if (msg_count_ == 1) {
      RCLCPP_INFO(this->get_logger(), "First lowstate message received");
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name = joint_names_;

    // Extract joint positions from motor states
    // Unitree motor order: FR(0,1,2), FL(3,4,5), RR(6,7,8), RL(9,10,11)
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);

    for (size_t i = 0; i < 12; ++i) {
      joint_state.position[i] = static_cast<double>(msg->motor_state[i].q);
      joint_state.velocity[i] = static_cast<double>(msg->motor_state[i].dq);
      joint_state.effort[i] = static_cast<double>(msg->motor_state[i].tau_est);
    }

    joint_pub_->publish(joint_state);

    if (msg_count_ % 1000 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Published %lu joint state messages", msg_count_);
    }
  }

  // Parameters
  std::string lowstate_topic_;
  std::string joint_states_topic_;

  // Joint names
  std::vector<std::string> joint_names_;

  // State
  size_t msg_count_{0};

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStateBridge>();
  
  RCLCPP_INFO(node->get_logger(), "JointStateBridge node starting...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}