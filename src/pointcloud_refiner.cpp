// Point Cloud Refinement Node for Go2
// Filters ground points and clusters obstacles for better RTAB-Map grid generation
// Based on tested code for Unitree Go2's low-mounted lidar

#include <memory>
#include <vector>
#include <set>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

class PointCloudRefiner : public rclcpp::Node
{
public:
    PointCloudRefiner() : Node("pointcloud_refiner")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/utlidar/cloud_deskewed");
        this->declare_parameter<std::string>("output_topic", "/refined_cloud");
        this->declare_parameter<double>("voxel_size", 0.05);
        this->declare_parameter<double>("cluster_tolerance", 0.1);
        this->declare_parameter<int>("min_cluster_size", 10);
        this->declare_parameter<int>("max_cluster_size", 25000);
        this->declare_parameter<double>("min_height", -0.3);  // Filter points below this (ground)
        this->declare_parameter<double>("max_height", 1.5);   // Filter points above this (ceiling)
        this->declare_parameter<bool>("use_intensity_marking", true);
        
        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        use_intensity_marking_ = this->get_parameter("use_intensity_marking").as_bool();

        // Subscriber for raw point cloud
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudRefiner::cloudCallback, this, std::placeholders::_1));

        // Publisher for refined point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "PointCloudRefiner started");
        RCLCPP_INFO(this->get_logger(), "  Input: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Height filter: %.2f to %.2f", min_height_, max_height_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
    double voxel_size_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double min_height_;
    double max_height_;
    bool use_intensity_marking_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *raw_cloud);

        if (raw_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Received empty point cloud");
            return;
        }

        // Step 1: Height filter to remove ground and ceiling points
        pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(raw_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*height_filtered);

        // Step 2: Voxel grid downsampling to remove duplicates/noise
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(height_filtered);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*filtered_cloud);

        if (filtered_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Point cloud empty after filtering");
            return;
        }

        // Step 3: Euclidean clustering to identify obstacles
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(filtered_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_cloud);
        ec.extract(cluster_indices);

        // Step 4: Create refined point cloud with intensity marking
        pcl::PointCloud<pcl::PointXYZI>::Ptr refined_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        
        if (use_intensity_marking_) {
            // Mark clustered points (obstacles) with high intensity
            // Mark non-clustered points (free space rays) with low intensity
            std::set<int> clustered_indices;
            
            for (const auto& indices : cluster_indices) {
                for (int index : indices.indices) {
                    clustered_indices.insert(index);
                    pcl::PointXYZI pt = filtered_cloud->points[index];
                    pt.intensity = 100.0f;  // High intensity = obstacle
                    refined_cloud->points.push_back(pt);
                }
            }
            
            // Add non-clustered points as free space indicators
            for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
                if (clustered_indices.find(i) == clustered_indices.end()) {
                    pcl::PointXYZI pt = filtered_cloud->points[i];
                    pt.intensity = 0.0f;  // Low intensity = free space
                    refined_cloud->points.push_back(pt);
                }
            }
        } else {
            // Just output clustered obstacle points (centroids)
            for (const auto& indices : cluster_indices) {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*filtered_cloud, indices.indices, centroid);
                pcl::PointXYZI pt;
                pt.x = centroid[0];
                pt.y = centroid[1];
                pt.z = centroid[2];
                pt.intensity = 100.0f;
                refined_cloud->points.push_back(pt);
            }
        }

        refined_cloud->width = refined_cloud->points.size();
        refined_cloud->height = 1;
        refined_cloud->is_dense = true;

        // Convert back to ROS2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*refined_cloud, output_msg);
        output_msg.header = msg->header;

        publisher_->publish(output_msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudRefiner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}