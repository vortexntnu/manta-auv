cmake_minimum_required(VERSION 3.8)
project(velocity_controller_lqr_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # Find Eigen3

# Include Directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create the executable
add_executable(${PROJECT_NAME} src/velocity_controller_lqr.cpp)

# Link the executable with its dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  vortex_msgs
  Eigen3
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Add ament package
ament_package()
