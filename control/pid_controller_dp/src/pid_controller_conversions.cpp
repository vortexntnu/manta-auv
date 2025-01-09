#include "pid_controller_dp/pid_controller_conversions.hpp"
#include <iostream>
#include "nav_msgs/msg/odometry.hpp"
#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Eta eta_convert_from_ros_to_eigen(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    types::Eta eta;
    eta.pos << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    eta.ori.w() = msg->pose.pose.orientation.w;
    eta.ori.x() = msg->pose.pose.orientation.x;
    eta.ori.y() = msg->pose.pose.orientation.y;
    eta.ori.z() = msg->pose.pose.orientation.z;

    return eta;
}

types::Nu nu_convert_from_ros_to_eigen(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    types::Nu nu;
    nu.linear_speed << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;
    nu.angular_speed << msg->twist.twist.angular.x, msg->twist.twist.angular.y,
        msg->twist.twist.angular.z;
    return nu;
}
