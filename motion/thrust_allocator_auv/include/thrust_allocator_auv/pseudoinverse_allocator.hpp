/**
 * @file pseudoinverse_allocator.hpp
 * @brief Contains the PseudoinverseAllocator class, which implements the
 * unweighted pseudoinverse-based allocator described in e.g. Fossen 2011
 * Handbook of Marine Craft Hydrodynamics and Motion Control (chapter 12.3.2).
 */

#ifndef VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_HPP
#define VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_HPP

#include <eigen3/Eigen/Eigen>

/**
 * @brief The PseudoinverseAllocator class calculates the allocated thrust given
 * the input torques using the pseudoinverse allocator.
 */
class PseudoinverseAllocator {
public:
  /**
   * @brief Constructor for the PseudoinverseAllocator class.
   *
   * @param T_pinv The pseudoinverse of the thruster configuration matrix.
   */
  explicit PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv);

  /**
   * @brief Calculates the allocated thrust given the input torques using the
   * pseudoinverse allocator.
   *
   * @param tau The input torques as a vector.
   * @return The allocated thrust as a vector.
   */
  Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd &tau);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::MatrixXd T_pinv;
};

#endif // VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_HPP
