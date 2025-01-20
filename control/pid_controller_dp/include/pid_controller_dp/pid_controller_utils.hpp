#ifndef PID_UTILS_HPP
#define PID_UTILS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "pid_controller_dp/typedefs.hpp"

// @brief Calculate the sine of an angle in degrees
// @param angle: Angle in degrees
// @return Smallest sine angle of the input
double ssa(double angle);

// @brief Convert a Float64MultiArray message to a 6x6 diagonal matrix
// @param msg: Float64MultiArray message containing 6 elements
// @return 6x6 diagonal matrix with the elements of the message on the diagonal
types::Matrix6d float64multiarray_to_diagonal_matrix6d(
    const std_msgs::msg::Float64MultiArray& msg);

// @brief Calculate the rotation matrix from a quaternion
// @param q: Quaternion represented as a 4D vector [w, x, y, z]
// @return 3x3 rotation matrix
// REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen 2021
// p.34 eq: 2.72
types::Matrix3d calculate_R_quat(const types::Eta& eta);

// @brief Calculate the transformation matrix from a quaternion
// @param q: Quaternion represented as a 4D vector [w, x, y, z]
// @return 4x3 transformation matrix
// REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen 2021
// p.35 eq: 2.78
types::Matrix4x3d calculate_T_quat(const types::Eta& eta);

// @brief Calculate the Jacobian matrix
// @param eta: 7D vector containing the vehicle pose [x, y, z, w, x, y, z]
// @return 7x6 Jacobian matrix
// REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen 2021
// p.36 eq: 2.83
types::J_transformation calculate_J(const types::Eta& eta);

// @brief Calculate the pseudo-inverse of the Jacobian matrix
// @param eta: 7D vector containing the vehicle pose [x, y, z, w, x, y, z]
// @return 6x7 pseudo-inverse Jacobian matrix
// REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen 2021
// p.34 eq: 2.72
types::Matrix6x7d calculate_J_sudo_inv(const types::Eta& eta);

// @brief Calculate the error between the desired and actual vehicle pose
// @param eta: 7D vector containing the actual vehicle pose [x, y, z, w, x, y,
// z]
// @param eta_d: 7D vector containing the desired vehicle pose [x, y, z, w, x,
// y, z]
// @return 7D vector containing the error between the desired and actual vehicle
// pose

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d);

// @brief Clamp the values between a minimum and maximum value
// @param values: Vector containing the values to be clamped
// @param min_val: Minimum value
// @param max_val: Maximum value
// @return Vector containing the clamped values
Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val);

// @brief Calculate the anti-windup term
// @param dt: Time step
// @param error: 7D vector containing the error between the desired and
// actual vehicle pose [x, y, z, w, x, y, z]
// @param integral: 7D vector containing the integral term of the PID
// controller [x, y, z, w, x, y, z]
// @return 7D vector containing the anti-windup term
types::Vector7d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector7d& integral);

// @brief Limit the input to the PID controller
// @param input: 6D vector containing the input to the PID controller [u, v, w,
// p, q, r]
// @return 6D vector containing the limited input to the PID controller [u, v,
// w, p, q, r]
types::Vector6d limit_input(const types::Vector6d& input);

#endif
