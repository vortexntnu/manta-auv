/**
 * @file typedefs.hpp
 * @brief Contains the typedef and structs for the controller.
 */

#ifndef VORTEX_DP_ADAPT_BACKSTEPPING_CONTROLLER_TYPEDEFS_H
#define VORTEX_DP_ADAPT_BACKSTEPPING_CONTROLLER_TYPEDEFS_H

#include <eigen3/Eigen/Dense>

namespace dp_types {
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 12> Matrix6x12d;
typedef Eigen::Matrix<double, 12, 6> Matrix12x6d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;

struct Eta {
    dp_types::Vector3d pos = dp_types::Vector3d::Zero();
    dp_types::Vector3d ori = dp_types::Vector3d::Zero();

    dp_types::Vector6d as_vector() const {
        dp_types::Vector6d vec;
        vec << pos, ori;
        return vec;
    }
};

struct Nu {
    dp_types::Vector3d linear_speed = dp_types::Vector3d::Zero();
    dp_types::Vector3d angular_speed = dp_types::Vector3d::Zero();

    dp_types::Vector6d as_vector() const {
        dp_types::Vector6d vec;
        vec << linear_speed, angular_speed;
        return vec;
    }
};

struct J_matrix {
    dp_types::Matrix3d R = dp_types::Matrix3d::Identity();
    dp_types::Matrix3d T = dp_types::Matrix3d::Identity();

    dp_types::Matrix6d as_matrix() const {
        dp_types::Matrix6d mat = dp_types::Matrix6d::Zero();
        mat.block<3, 3>(0, 0) = R;
        mat.block<3, 3>(3, 3) = T;
        return mat;
    }
};

}  // namespace dp_types

#endif
