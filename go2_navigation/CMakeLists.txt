cmake_minimum_required(VERSION 3.20)
project(go2_navigation)

find_package(ament_cmake REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odom_tf_bridge src/odom_tf_bridge.cpp)
ament_target_dependencies(odom_tf_bridge rclcpp nav_msgs tf2_ros geometry_msgs)

add_executable(go2SportTest src/go2SportTest.cpp)

# Unitree headers
target_include_directories(go2SportTest PRIVATE
  /opt/unitree_robotics/include
  /opt/unitree_robotics/include/ddscxx/
)

# Unitree libs
target_link_directories(go2SportTest PRIVATE
  /opt/unitree_robotics/lib
)

# Link Unitree SDK2
target_link_libraries(go2SportTest
  unitree_sdk2
  ddscxx
  ddsc
  pthread
)

ament_target_dependencies(go2SportTest)

install(TARGETS 
  go2SportTest
  odom_tf_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  meshes
  config
  xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
