#include "pid_controller_dp/pid_controller_utils.hpp"
#include <algorithm>
#include <iostream>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Matrix6d float64multiarray_to_diagonal_matrix6d(
    const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() != 6) {
        throw std::runtime_error(
            "Float64MultiArray message must have exactly 6 elements.");
    }

    Eigen::Map<const types::Vector6d> vec(msg.data.data());
    types::Matrix6d matrix = vec.asDiagonal();

    return matrix;
}

types::Matrix3d calculate_R_quat(const types::Eta& eta) {
    return eta.ori.normalized().toRotationMatrix();
}

types::Matrix4x3d calculate_T_quat(const types::Eta& eta) {
    types::Quaterniond quaternion_norm = eta.ori.normalized();

    double w = quaternion_norm.w();
    double x = quaternion_norm.x();
    double y = quaternion_norm.y();
    double z = quaternion_norm.z();

    types::Matrix4x3d transformation_matrix;

    transformation_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;

    return transformation_matrix * 0.5;
}

types::Matrix6x7d calculate_J_sudo_inv(const types::Eta& eta) {
    types::Eta eta_norm;

    eta_norm.pos = eta.pos;
    eta_norm.ori = eta.ori;

    types::Matrix3d R = calculate_R_quat(eta_norm);
    types::Matrix4x3d T = calculate_T_quat(eta_norm);

    types::J_transformation J;
    J.R = R;
    J.T = T;

    types::Matrix6x7d J_transpose = J.as_matrix().transpose();
    types::Matrix6x7d J_pseudo_inv =
        (J_transpose * J.as_matrix()).inverse() * J_transpose;

    return J_pseudo_inv;
}

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d) {
    types::Eta eta_error;

    eta_error.pos = eta.pos - eta_d.pos;
    eta_error.ori = eta_d.ori.conjugate() * eta.ori;

    eta_error.ori = eta_error.ori.normalized();

    return eta_error;
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val) {
    return values.cwiseMax(min_val).cwiseMin(max_val);
}

types::Vector7d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector7d& integral) {
    types::Eta error_norm;

    error_norm.pos = error.pos;
    error_norm.ori = error.ori;

    types::Vector7d integral_anti_windup =
        integral + (error_norm.as_vector() * dt);

    integral_anti_windup = clamp_values(integral_anti_windup, -80.0, 80.0);
    return integral_anti_windup;
}

types::Vector6d limit_input(const types::Vector6d& input) {
    types::Vector6d limited_input = input;

    limited_input = clamp_values(input, -85.0, 85.0);

    return limited_input;
}
