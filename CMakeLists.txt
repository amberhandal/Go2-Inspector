cmake_minimum_required(VERSION 3.20)
project(go2_navigation)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

# ============================================================================
# Path to unitree_ros2 example source
# Layout: go2_humble_ws/src/
#           ├── go2Inspector/go2_navigation/  (this package)
#           └── unitree_ros2/example/         (unitree example)
# ============================================================================
set(UNITREE_ROS2_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros2/example")

# Verify the path exists
if(NOT EXISTS "${UNITREE_ROS2_EXAMPLE_DIR}/src/include/common/ros2_sport_client.h")
  message(FATAL_ERROR 
    "Cannot find unitree_ros2 example at: ${UNITREE_ROS2_EXAMPLE_DIR}\n"
    "Expected structure: unitree_ros2/example/src/include/common/ros2_sport_client.h\n"
    "Please ensure unitree_ros2 is cloned in your workspace src folder.\n"
    "Your workspace layout should be:\n"
    "  go2_humble_ws/src/\n"
    "    ├── go2Inspector/go2_navigation/  (this package)\n"
    "    └── unitree_ros2/example/         (unitree)\n")
endif()

message(STATUS "Found unitree_ros2 example at: ${UNITREE_ROS2_EXAMPLE_DIR}")

# ============================================================================
# Unitree Sport Client Library (built from unitree_ros2 example source)
# ============================================================================
add_library(unitree_sport_client
  ${UNITREE_ROS2_EXAMPLE_DIR}/src/src/common/ros2_sport_client.cpp
)

# The unitree source uses #include "ros2_sport_client.h" (no common/ prefix)
# So we need to include both the base include dir AND the common subfolder
target_include_directories(unitree_sport_client PUBLIC
  ${UNITREE_ROS2_EXAMPLE_DIR}/src/include
  ${UNITREE_ROS2_EXAMPLE_DIR}/src/include/common
)

target_compile_features(unitree_sport_client PUBLIC cxx_std_20)

ament_target_dependencies(unitree_sport_client
  rclcpp
  unitree_api
  unitree_go
)

# ============================================================================
# Executables
# ============================================================================

# --- cmdvel_to_sport_bridge ---
add_executable(cmdvel_to_sport_bridge
  src/cmdvel_to_sport_bridge.cpp
)

target_include_directories(cmdvel_to_sport_bridge PRIVATE
  ${UNITREE_ROS2_EXAMPLE_DIR}/src/include
  ${UNITREE_ROS2_EXAMPLE_DIR}/src/include/common
)

target_compile_features(cmdvel_to_sport_bridge PRIVATE cxx_std_20)

target_link_libraries(cmdvel_to_sport_bridge
  unitree_sport_client
)

ament_target_dependencies(cmdvel_to_sport_bridge
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

# --- odom_tf_bridge ---
add_executable(odom_tf_bridge 
  src/odom_tf_bridge.cpp
)

target_compile_features(odom_tf_bridge PRIVATE cxx_std_20)

ament_target_dependencies(odom_tf_bridge 
  rclcpp 
  nav_msgs 
  tf2_ros 
  geometry_msgs
)

# --- navigation_node ---
add_executable(navigation_node
  src/navigation_node.cpp
)

target_compile_features(navigation_node PRIVATE cxx_std_20)

ament_target_dependencies(navigation_node
  rclcpp
  geometry_msgs
  std_srvs
)

add_executable(pointcloud_refiner src/pointcloud_refiner.cpp)
ament_target_dependencies(pointcloud_refiner
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pointcloud_refiner
  ${PCL_LIBRARIES}
)
target_include_directories(pointcloud_refiner PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# Frontier Explorer Node
add_executable(frontier_explorer src/frontier_explorer.cpp)
ament_target_dependencies(frontier_explorer
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  visualization_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# --- test_movement ---
add_executable(test_movement
  src/test_movement_node.cpp
)

target_compile_features(test_movement PRIVATE cxx_std_20)

ament_target_dependencies(test_movement
  rclcpp
  geometry_msgs
)

# --- joint_state_bridge ---
add_executable(joint_state_bridge
  src/joint_state_bridge.cpp
)

target_compile_features(joint_state_bridge PRIVATE cxx_std_20)

ament_target_dependencies(joint_state_bridge
  rclcpp
  sensor_msgs
  unitree_go
)

# --- pointcloud_to_scan ---
add_executable(pointcloud_to_scan
  src/pointcloud_to_scan.cpp
)

target_compile_features(pointcloud_to_scan PRIVATE cxx_std_20)

ament_target_dependencies(pointcloud_to_scan
  rclcpp
  sensor_msgs
)

# ============================================================================
# Install
# ============================================================================

install(TARGETS
  cmdvel_to_sport_bridge
  odom_tf_bridge
  navigation_node
  test_movement
  frontier_explorer
  joint_state_bridge
  pointcloud_refiner
  pointcloud_to_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointcloud_restamper.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Testing (optional)
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()