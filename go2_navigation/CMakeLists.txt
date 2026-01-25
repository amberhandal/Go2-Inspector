cmake_minimum_required(VERSION 3.20)
project(go2_navigation)

find_package(ament_cmake REQUIRED)

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

# Link Unitree SDK2 (name may differ; see below)
target_link_libraries(go2SportTest
  unitree_sdk2
  ddscxx
  ddsc
  pthread
)

ament_target_dependencies(go2SportTest)

install(TARGETS go2SportTest
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
