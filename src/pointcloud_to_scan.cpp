/**
 * @file pointcloud_to_scan.cpp
 * @brief Converts 3D PointCloud2 to 2D LaserScan for Nav2
 * 
 * Takes a horizontal slice of the point cloud and converts it to 
 * a LaserScan message for use with SLAM Toolbox and Nav2.
 */

#include <memory>
#include <string>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudToScan : public rclcpp::Node
{
public:
  PointCloudToScan()
  : Node("pointcloud_to_scan")
  {
    // Parameters
    this->declare_parameter<std::string>("cloud_topic", "/utlidar/cloud");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("scan_frame", "utlidar_lidar");
    
    // Height filter - only consider points in this Z range (relative to sensor)
    this->declare_parameter<double>("min_height", -0.3);  // meters below sensor
    this->declare_parameter<double>("max_height", 0.3);   // meters above sensor
    
    // Scan parameters
    this->declare_parameter<double>("range_min", 0.1);    // meters
    this->declare_parameter<double>("range_max", 30.0);   // meters
    this->declare_parameter<double>("angle_min", -M_PI); // -180 degrees
    this->declare_parameter<double>("angle_max", M_PI);  // +180 degrees
    this->declare_parameter<double>("angle_increment", M_PI / 360.0);  // 0.5 degree resolution

    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    scan_frame_ = this->get_parameter("scan_frame").as_string();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();

    // Calculate number of scan rays
    num_ranges_ = static_cast<size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));

    // Publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);

    // Subscriber with QoS matching Unitree
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos,
      std::bind(&PointCloudToScan::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloudToScan initialized:");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Frame: %s", scan_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Height filter: [%.2f, %.2f] m", min_height_, max_height_);
    RCLCPP_INFO(this->get_logger(), "  Range: [%.2f, %.2f] m", range_min_, range_max_);
    RCLCPP_INFO(this->get_logger(), "  Scan rays: %zu", num_ranges_);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    msg_count_++;
    
    if (msg_count_ == 1) {
      RCLCPP_INFO(this->get_logger(), "First point cloud received");
    }

    // Create LaserScan message
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = scan_frame_;
    
    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 1.0 / 15.0;  // ~15 Hz
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;

    // Initialize ranges with infinity (no detection)
    scan_msg.ranges.resize(num_ranges_, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(num_ranges_, 0.0f);

    // Iterate through point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      // Skip invalid points
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // Height filter
      if (z < min_height_ || z > max_height_) {
        continue;
      }

      // Calculate range and angle
      const float range = std::sqrt(x * x + y * y);
      
      // Range filter
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const float angle = std::atan2(y, x);
      
      // Angle filter
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }

      // Calculate index in scan array
      const size_t index = static_cast<size_t>(
        (angle - angle_min_) / angle_increment_);
      
      if (index < num_ranges_) {
        // Keep the closest point for each angle
        if (range < scan_msg.ranges[index]) {
          scan_msg.ranges[index] = range;
        }
      }
    }

    scan_pub_->publish(scan_msg);
  }

  // Parameters
  std::string cloud_topic_;
  std::string scan_topic_;
  std::string scan_frame_;
  double min_height_;
  double max_height_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  size_t num_ranges_;

  // State
  size_t msg_count_{0};

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToScan>();
  
  RCLCPP_INFO(node->get_logger(), "PointCloudToScan node starting...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}