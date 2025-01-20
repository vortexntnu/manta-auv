#ifndef DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP
#define DP_ADAPT_BACKS_CONTROLLER_UTILS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "dp_adapt_backs_controller/typedefs.hpp"
#include "typedefs.hpp"

// @brief Calculate the sine of an angle in degrees
// @param angle: Angle in degrees
// @return Smallest sine angle of the input
double ssa(double angle);

// @brief construct a skew symmetric matrix from a 3D vector
// @param vec: 3D vector
// @return 3x3 skew symmetric matrix
dp_types::Matrix3d skew_symmetric(const dp_types::Vector3d& vec);

// @brief Calculate the error in eta
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param eta_d: 6D vector containing the desired vehicle pose [x, y, z, roll,
// pitch, yaw]
// @return 6D vector containing the error between the desired and actual vehicle
// pose
dp_types::Eta error_eta(const dp_types::Eta& eta, const dp_types::Eta& eta_d);

// @brief Quaternion to Euler angle conversion
// @param q: Quaternion represented as a 4D vector [w, x, y, z]
// @return 3D vector containing the Euler angles [roll, pitch, yaw]
dp_types::Vector3d quat_to_euler(const double w,
                                 const double x,
                                 const double y,
                                 const double z);

// @brief Calculate the rotation matrix from a Euler angle
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @return 3x3 rotation matrix
dp_types::Matrix3d calculate_R(const dp_types::Eta& eta);

// @brief Calculate the transformation matrix from a Euler angle
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @return 3x3 transformation matrix
dp_types::Matrix3d calculate_T(const dp_types::Eta& eta);

// @brief Calculate the derivative of the rotation matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the rotation matrix
dp_types::Matrix3d calculate_R_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu);

// @brief Calculate the derivative of the transformation matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 3x3 derivative of the transformation matrix
dp_types::Matrix3d calculate_T_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu);

// @brief Calculate the Jacobian matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @return 6x6 Jacobian matrix
dp_types::Matrix6d calculate_J(const dp_types::Eta& eta);

// @brief Calculate the pseudo-inverse of the Jacobian matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @return 6x6 pseudo-inverse Jacobian matrix
dp_types::Matrix6d calculate_J_sudo_inv(const dp_types::Eta& eta);

// @brief calculate the derivative of the Jacobian matrix
// @param eta: 6D vector containing the vehicle pose [x, y, z, roll, pitch, yaw]
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 6x6 derivative of the Jacobian matrix
dp_types::Matrix6d calculate_J_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu);

// @brief Calculate the coriolis matrix
// @param m: mass of the vehicle
// @param r_b_bg: 3D vector of the body frame to the center of gravity
// @param nu_2: 3D vector contating angular velocity of the vehicle
// @param I_b : 3D matrix containing the inertia matrix
// @return 6x6 coriolis matrix
dp_types::Matrix6d calculate_C(double m,
                               const dp_types::Vector3d& r_b_bg,
                               const dp_types::Nu& nu_2,
                               const dp_types::Matrix3d& I_b);

// @brief Calculate the damping matrix for the adaptive backstepping controller
// @param nu: 6D vector containing the vehicle velocity [u, v, w, p, q, r]
// @return 6x6 damping matrix
dp_types::Matrix6x12d calculate_Y_v(const dp_types::Nu& nu);

#endif
