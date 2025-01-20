/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <eigen3/Eigen/Dense>

namespace types {
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 4, 3> Matrix4x3d;
typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
typedef Eigen::Matrix<double, 6, 7> Matrix6x7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Quaterniond Quaterniond;

struct Eta {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond ori = Eigen::Quaterniond::Identity();

    types::Vector7d as_vector() const {
        types::Vector7d vec;
        vec << pos, ori.w(), ori.x(), ori.y(), ori.z();
        return vec;
    }
};

struct Nu {
    Eigen::Vector3d linear_speed = types::Vector3d::Zero();
    Eigen::Vector3d angular_speed = types::Vector3d::Zero();

    types::Vector6d as_vector() const {
        types::Vector6d vec;
        vec << linear_speed, angular_speed;
        return vec;
    }
};

struct J_transformation {
    Eigen::Matrix3d R = types::Matrix3d::Identity();
    types::Matrix4x3d T = types::Matrix4x3d::Zero();

    types::Matrix7x6d as_matrix() const {
        types::Matrix7x6d mat = types::Matrix7x6d::Zero();
        mat.block<3, 3>(0, 0) = R;
        mat.block<4, 3>(3, 3) = T;
        return mat;
    }
};
}  // namespace types

#endif
