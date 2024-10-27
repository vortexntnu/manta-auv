#include "../include/velocity_controller_lqr_cpp/velocity_controller_lqr_cpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityLQRNode>());
  rclcpp::shutdown();
  return 0;
}
