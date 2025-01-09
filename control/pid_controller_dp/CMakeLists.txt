cmake_minimum_required(VERSION 3.8)
project(pid_controller_dp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)

add_executable(pid_controller_node
  src/pid_controller_node.cpp
  src/pid_controller_ros.cpp
  src/pid_controller.cpp
  src/pid_controller_utils.cpp
  src/pid_controller_conversions.cpp
)

ament_target_dependencies(pid_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
)

install(TARGETS
  pid_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
