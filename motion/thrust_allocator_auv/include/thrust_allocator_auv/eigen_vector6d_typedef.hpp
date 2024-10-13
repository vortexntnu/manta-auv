/**
 * @file eigen_vector6d_typedef.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector.
 */

#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <eigen3/Eigen/Dense>

namespace Eigen {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

#endif // VORTEX_EIGEN_TYPEDEFS_H
