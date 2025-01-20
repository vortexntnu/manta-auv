#ifndef PID_CONTROLLER_CONVERSIONS_HPP
#define PID_CONTROLLER_CONVERSIONS_HPP

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "pid_controller_dp/typedefs.hpp"

types::Eta eta_convert_from_ros_to_eigen(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

types::Nu nu_convert_from_ros_to_eigen(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

#endif
