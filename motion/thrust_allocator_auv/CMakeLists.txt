cmake_minimum_required(VERSION 3.8)
project(thrust_allocator_auv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/thrust_allocator_auv_node.cpp
  src/thrust_allocator_ros.cpp
  src/pseudoinverse_allocator.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  vortex_msgs
  Eigen3
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
