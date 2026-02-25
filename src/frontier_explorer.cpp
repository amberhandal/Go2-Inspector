// Frontier exploration node for Go2
// Subscribes to /map, finds frontiers, sends Nav2 goals to explore unknown areas

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum class ExploreState
{
    IDLE,
    EXPLORING,
    NAVIGATING,
    STOPPED
};

class FrontierExplorer : public rclcpp::Node
{
public:
    FrontierExplorer() : Node("frontier_explorer")
    {
        // Parameters
        this->declare_parameter("min_frontier_size", 5);
        this->declare_parameter("exploration_radius", 10.0);
        this->declare_parameter("min_goal_distance", 0.5);
        this->declare_parameter("goal_timeout", 60.0);
        
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
        exploration_radius_ = this->get_parameter("exploration_radius").as_double();
        min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();
        goal_timeout_ = this->get_parameter("goal_timeout").as_double();

        // Services
        start_srv_ = create_service<std_srvs::srv::Empty>(
            "explore/start",
            std::bind(&FrontierExplorer::start_callback, this, std::placeholders::_1, std::placeholders::_2));
        stop_srv_ = create_service<std_srvs::srv::Empty>(
            "explore/stop",
            std::bind(&FrontierExplorer::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribers
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&FrontierExplorer::map_callback, this, std::placeholders::_1));

        // Publishers
        frontier_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers", 10);
        goal_pub_ = create_publisher<visualization_msgs::msg::Marker>("/exploration_goal_marker", 10);

        // Nav2 action client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer
        timer_ = create_wall_timer(500ms, std::bind(&FrontierExplorer::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Frontier Explorer initialized");
        RCLCPP_INFO(this->get_logger(), "Call 'ros2 service call /explore/start std_srvs/srv/Empty' to start exploring");
    }

private:
    // Parameters
    int min_frontier_size_;
    double exploration_radius_;
    double min_goal_distance_;
    double goal_timeout_;

    // State
    ExploreState state_ = ExploreState::IDLE;
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;
    std::vector<std::pair<double, double>> frontiers_;
    std::pair<double, double> current_goal_ = {0.0, 0.0};
    bool goal_active_ = false;
    rclcpp::Time goal_start_time_;

    // Robot pose
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    // ROS interfaces
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    void start_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Starting exploration!");
        state_ = ExploreState::EXPLORING;
        goal_active_ = false;
    }

    void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping exploration!");
        state_ = ExploreState::STOPPED;
        goal_active_ = false;
        // Cancel current navigation
        if (nav_client_->action_server_is_ready()) {
            nav_client_->async_cancel_all_goals();
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_received_ = true;
    }

    void update_robot_pose()
    {
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, 1s);
            robot_x_ = transform.transform.translation.x;
            robot_y_ = transform.transform.translation.y;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Could not get robot pose: %s", ex.what());
        }
    }

    std::vector<std::pair<double, double>> detect_frontiers()
    {
        std::vector<std::pair<double, double>> frontier_centroids;
        
        if (!map_received_) {
            return frontier_centroids;
        }

        int width = map_.info.width;
        int height = map_.info.height;
        double resolution = map_.info.resolution;
        double origin_x = map_.info.origin.position.x;
        double origin_y = map_.info.origin.position.y;

        // Find frontier cells (free cells adjacent to unknown cells)
        std::vector<bool> is_frontier(width * height, false);
        std::vector<std::pair<int, int>> frontier_cells;

        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                int idx = y * width + x;
                
                // Check if cell is free (0 = free, -1 = unknown, 100 = occupied)
                if (map_.data[idx] == 0) {
                    // Check if adjacent to unknown
                    bool adjacent_to_unknown = false;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nidx = (y + dy) * width + (x + dx);
                            if (map_.data[nidx] == -1) {
                                adjacent_to_unknown = true;
                                break;
                            }
                        }
                        if (adjacent_to_unknown) break;
                    }
                    
                    if (adjacent_to_unknown) {
                        is_frontier[idx] = true;
                        frontier_cells.push_back({x, y});
                    }
                }
            }
        }

        // Cluster frontier cells using BFS
        std::vector<bool> visited(width * height, false);
        
        for (const auto& start : frontier_cells) {
            int start_idx = start.second * width + start.first;
            if (visited[start_idx]) continue;
            
            // BFS to find connected frontier cells
            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> cluster;
            queue.push(start);
            visited[start_idx] = true;
            
            while (!queue.empty()) {
                auto [cx, cy] = queue.front();
                queue.pop();
                cluster.push_back({cx, cy});
                
                // Check 8-connected neighbors
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx;
                        int ny = cy + dy;
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                        
                        int nidx = ny * width + nx;
                        if (!visited[nidx] && is_frontier[nidx]) {
                            visited[nidx] = true;
                            queue.push({nx, ny});
                        }
                    }
                }
            }
            
            // Only keep clusters larger than minimum size
            if (static_cast<int>(cluster.size()) >= min_frontier_size_) {
                // Calculate centroid
                double sum_x = 0, sum_y = 0;
                for (const auto& [cx, cy] : cluster) {
                    sum_x += cx;
                    sum_y += cy;
                }
                double centroid_x = origin_x + (sum_x / cluster.size()) * resolution;
                double centroid_y = origin_y + (sum_y / cluster.size()) * resolution;
                
                frontier_centroids.push_back({centroid_x, centroid_y});
            }
        }

        return frontier_centroids;
    }

    std::pair<double, double> select_best_frontier(const std::vector<std::pair<double, double>>& frontiers)
    {
        if (frontiers.empty()) {
            return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
        }

        // Select closest frontier that is at least min_goal_distance away
        double best_dist = std::numeric_limits<double>::max();
        std::pair<double, double> best_frontier = frontiers[0];
        
        for (const auto& [fx, fy] : frontiers) {
            double dist = std::hypot(fx - robot_x_, fy - robot_y_);
            
            // Skip frontiers too close
            if (dist < min_goal_distance_) continue;
            
            // Skip frontiers too far
            if (dist > exploration_radius_) continue;
            
            if (dist < best_dist) {
                best_dist = dist;
                best_frontier = {fx, fy};
            }
        }

        return best_frontier;
    }

    void send_nav_goal(double x, double y)
    {
        if (!nav_client_->action_server_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Nav2 action server not ready");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending nav goal: (%.2f, %.2f)", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const GoalHandleNavigate::WrappedResult& result) {
            goal_active_ = false;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown result code");
                    break;
            }
        };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        current_goal_ = {x, y};
        goal_active_ = true;
        goal_start_time_ = this->now();
        state_ = ExploreState::NAVIGATING;
    }

    void publish_frontier_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Clear old markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        int id = 0;
        for (const auto& [fx, fy] : frontiers_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "frontiers";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = fx;
            marker.pose.position.y = fy;
            marker.pose.position.z = 0.1;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(marker);
        }
        
        frontier_pub_->publish(marker_array);
    }

    void publish_goal_marker()
    {
        if (!goal_active_) return;
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "exploration_goal";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = current_goal_.first;
        marker.pose.position.y = current_goal_.second;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        goal_pub_->publish(marker);
    }

    void timer_callback()
    {
        update_robot_pose();
        
        // Detect and publish frontiers
        frontiers_ = detect_frontiers();
        publish_frontier_markers();
        publish_goal_marker();

        // State machine
        switch (state_) {
            case ExploreState::IDLE:
                // Do nothing, waiting for start command
                break;
                
            case ExploreState::EXPLORING:
                {
                    if (!map_received_) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                             "Waiting for map data...");
                        return;
                    }
                    
                    if (frontiers_.empty()) {
                        // Check if map has any free space
                        int free_cells = 0;
                        for (const auto& cell : map_.data) {
                            if (cell == 0) free_cells++;
                        }
                        
                        if (free_cells < 10) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                                 "Map has no free space yet (%d free cells). Move the robot to start mapping.", free_cells);
                            return;
                        }
                        
                        RCLCPP_INFO(this->get_logger(), "No frontiers found - exploration complete!");
                        state_ = ExploreState::IDLE;
                        return;
                    }
                    
                    auto best = select_best_frontier(frontiers_);
                    if (std::isnan(best.first)) {
                        RCLCPP_WARN(this->get_logger(), "No valid frontier found");
                        return;
                    }
                    
                    send_nav_goal(best.first, best.second);
                }
                break;
                
            case ExploreState::NAVIGATING:
                {
                    // Check for timeout
                    if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
                        RCLCPP_WARN(this->get_logger(), "Goal timed out, finding new frontier");
                        nav_client_->async_cancel_all_goals();
                        goal_active_ = false;
                        state_ = ExploreState::EXPLORING;
                        return;
                    }
                    
                    // Check if goal completed
                    if (!goal_active_) {
                        state_ = ExploreState::EXPLORING;
                    }
                }
                break;
                
            case ExploreState::STOPPED:
                // Do nothing
                break;
        }

        // Log status
        if (state_ == ExploreState::EXPLORING || state_ == ExploreState::NAVIGATING) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Exploring... Frontiers: %zu, Robot: (%.2f, %.2f)",
                                  frontiers_.size(), robot_x_, robot_y_);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}