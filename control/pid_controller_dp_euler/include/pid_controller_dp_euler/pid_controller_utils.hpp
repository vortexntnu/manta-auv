#ifndef PID_UTILS_HPP
#define PID_UTILS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <pid_controller_dp_euler/typedefs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

Matrix6d float64multiarray_to_diagonal_matrix6d(
    const std_msgs::msg::Float64MultiArray& msg);

double ssa(double angle);

Eta apply_ssa(const Eta& eta);

Matrix6d calculate_j(const Eta& eta);

Vector6d anti_windup(const double dt,
                     const Vector6d& error,
                     const Vector6d& integral);

Vector6d clamp_values(const Vector6d& values, double min_val, double max_val);

Vector6d limit_input(const Vector6d& input);

#endif
