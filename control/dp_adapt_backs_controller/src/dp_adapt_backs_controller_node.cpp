#include "dp_adapt_backs_controller/dp_adapt_backs_controller_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Started DP Adaptive Backstepping Controller Node");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "-- Owner by Talha Nauman Choudhry --");
    rclcpp::spin(std::make_shared<DPAdaptBacksControllerNode>());
    rclcpp::shutdown();
    return 0;
}
