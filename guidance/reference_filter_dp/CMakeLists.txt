cmake_minimum_required(VERSION 3.8)
project(reference_filter_dp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(reference_filter_node
  src/reference_filter_node.cpp
  src/reference_filter_ros.cpp
  src/reference_filter_utils.cpp
  src/reference_filter.cpp
)

ament_target_dependencies(reference_filter_node
  rclcpp
  rclcpp_action
  geometry_msgs
  vortex_msgs
  Eigen3
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  reference_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
