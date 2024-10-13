/**
 * @file thrust_allocator_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module.
 */

#ifndef VORTEX_ALLOCATOR_UTILS_HPP
#define VORTEX_ALLOCATOR_UTILS_HPP

#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <vortex_msgs/msg/thruster_forces.hpp>

/**
 * @brief Check if the matrix has any NaN or INF elements.
 *
 * @tparam Derived The type of the matrix.
 * @param M The matrix to check.
 * @return true if the matrix has any NaN or INF elements, false otherwise.
 */
template <typename Derived>
inline bool is_invalid_matrix(const Eigen::MatrixBase<Derived> &M) {
  bool has_nan = !(M.array() == M.array()).all();
  bool has_inf = M.array().isInf().any();
  return has_nan || has_inf;
}

inline Eigen::MatrixXd calculate_thrust_allocation_matrix(
    const Eigen::MatrixXd &thruster_force_direction,
    const Eigen::MatrixXd &thruster_position,
    const Eigen::Vector3d &center_of_mass) {
  // Initialize thrust allocation matrix
  Eigen::MatrixXd thrust_allocation_matrix = Eigen::MatrixXd::Zero(6, 8);

  // Calculate thrust and moment contributions from each thruster
  for (int i = 0; i < thruster_position.cols(); i++) {
    Eigen::Vector3d pos =
        thruster_position.col(i); // Thrust vector in body frame
    Eigen::Vector3d F =
        thruster_force_direction.col(i); // Position vector in body frame

    // Calculate position vector relative to the center of mass
    pos -= center_of_mass;

    // Fill in the thrust allocation matrix
    thrust_allocation_matrix.block<3, 1>(0, i) = F;
    thrust_allocation_matrix.block<3, 1>(3, i) = pos.cross(F);
  }

  thrust_allocation_matrix = thrust_allocation_matrix.array();
  return thrust_allocation_matrix;
}

/**
 * @brief Calculates the right pseudoinverse of the given matrix.
 *
 * @param M The matrix to calculate the pseudoinverse of.
 * @throws char* if the pseudoinverse is invalid.
 * @return The pseudoinverse of the given matrix.
 */
inline Eigen::MatrixXd calculate_right_pseudoinverse(const Eigen::MatrixXd &T) {
  Eigen::MatrixXd pseudoinverse = T.transpose() * (T * T.transpose()).inverse();
  if (is_invalid_matrix(pseudoinverse)) {
    throw std::runtime_error("Invalid Pseudoinverse Calculated");
  }
  return pseudoinverse;
}

/**
 * @brief Saturates the values of a given Eigen vector between a minimum and
 * maximum value.
 *
 * @param vec The Eigen vector to be saturated.
 * @param min The minimum value to saturate the vector values to.
 * @param max The maximum value to saturate the vector values to.
 * @return True if all vector values are within the given range, false
 * otherwise.
 */
inline bool saturate_vector_values(Eigen::VectorXd &vec, double min,
                                   double max) {
  bool all_values_in_range =
      std::all_of(vec.begin(), vec.end(),
                  [min, max](double val) { return val >= min && val <= max; });

  std::transform(vec.begin(), vec.end(), vec.begin(), [min, max](double val) {
    return std::min(std::max(val, min), max);
  });

  return all_values_in_range;
}

/**
 * @brief Converts an Eigen VectorXd to a vortex_msgs::msg::ThrusterForces
 * message.
 *
 * @param u The Eigen VectorXd to be converted.
 * @param msg The vortex_msgs::msg::ThrusterForces message to store the
 * converted values.
 */
inline void array_eigen_to_msg(const Eigen::VectorXd &u,
                               vortex_msgs::msg::ThrusterForces &msg) {
  int r = u.size();
  std::vector<double> u_vec(r);
  std::copy_n(u.begin(), r, u_vec.begin());
  msg.thrust = u_vec;
}

/**
 * @brief Converts a 1D array of doubles to a 2D Eigen matrix.
 *
 * @param matrix The 1D array of doubles to be converted.
 * @param rows The number of rows in the resulting Eigen matrix.
 * @param cols The number of columns in the resulting Eigen matrix.
 * @return The resulting Eigen matrix.
 */
inline Eigen::MatrixXd
double_array_to_eigen_matrix(const std::vector<double> &matrix, int rows,
                             int cols) {
  return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                        Eigen::RowMajor>>(matrix.data(), rows,
                                                          cols);
}

inline Eigen::Vector3d
double_array_to_eigen_vector3d(const std::vector<double> &vector) {
  // Ensure the input vector has exactly three elements
  if (vector.size() != 3) {
    throw std::invalid_argument("Input vector must have exactly 3 elements");
  }

  // Map the vector to Eigen::Vector3d
  return Eigen::Map<const Eigen::Vector3d>(vector.data());
}

#endif // VORTEX_ALLOCATOR_UTILS_HPP
