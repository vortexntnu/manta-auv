#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"
#include <vector>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Issue 1: Inefficient logging - using std::string instead of a string literal
    std::string log_message = "Started thruster_interface_auv_node";  // Unnecessary allocation
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), log_message.c_str());

    // Issue 2: Implicit casting from int to float
    int update_rate = 10;
    float update_period = 1.0 / update_rate;  // Implicit narrowing cast

    // Issue 3: Unused variable - clang-tidy will warn if a variable is unused
    std::vector<int> unused_vector = {1, 2, 3};  // Unused variable

    // Issue 4: Avoid creating shared_ptr in function arguments - inefficient and can lead to multiple instances
    rclcpp::spin(std::make_shared<ThrusterInterfaceAUVNode>());

    rclcpp::shutdown();
    return 0;
}
