cmake_minimum_required(VERSION 3.8)
project(acoustics_data_record)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/acoustics_data_record_lib.py
  ${PROJECT_NAME}/acoustics_data_record_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
