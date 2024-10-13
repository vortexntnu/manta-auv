#include "thrust_allocator_auv/thrust_allocator_ros.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto allocator = std::make_shared<ThrustAllocator>();
  RCLCPP_INFO(allocator->get_logger(), "Thrust allocator initiated");
  rclcpp::spin(allocator);
  rclcpp::shutdown();
  return 0;
}
