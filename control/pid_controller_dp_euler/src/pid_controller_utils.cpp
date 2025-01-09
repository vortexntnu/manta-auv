#include <pid_controller_dp_euler/pid_controller_utils.hpp>

Matrix6d float64multiarray_to_diagonal_matrix6d(
    const std_msgs::msg::Float64MultiArray& msg) {
    Matrix6d matrix = Matrix6d::Zero();

    if (msg.data.size() != 6) {
        throw std::runtime_error(
            "Float64MultiArray message must have exactly 6 elements.");
    }

    for (size_t i = 0; i < 6; ++i) {
        matrix(i, i) = msg.data[i];
    }

    return matrix;
}

double ssa(double angle) {
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
}

Eta apply_ssa(const Eta& eta) {
    Eta eta_ssa = eta;

    eta_ssa.roll = ssa(eta.roll);
    eta_ssa.pitch = ssa(eta.pitch);
    eta_ssa.yaw = ssa(eta.yaw);

    return eta_ssa;
}

Matrix6d calculate_j(const Eta& eta) {
    Matrix3d rotation_matrix = eta.as_rotation_matrix();
    Matrix3d transformation_matrix = eta.as_transformation_matrix();

    Matrix6d j = Matrix6d::Zero();
    j.block<3, 3>(0, 0) = rotation_matrix;
    j.block<3, 3>(3, 3) = transformation_matrix;

    return j;
}

Vector6d anti_windup(const double dt,
                     const Vector6d& error,
                     const Vector6d& integral) {
    Vector6d integral_anti_windup = integral + (error * dt);

    integral_anti_windup = clamp_values(integral_anti_windup, -30.0, 30.0);
    return integral_anti_windup;
}

Vector6d clamp_values(const Vector6d& values, double min_val, double max_val) {
    auto clamp = [min_val, max_val](double x) {
        return std::clamp(x, min_val, max_val);
    };
    Vector6d clamped_values = values.unaryExpr(clamp);

    return clamped_values;
}

Vector6d limit_input(const Vector6d& input) {
    Vector6d limited_input = input;

    limited_input = clamp_values(input, -95.0, 95.0);

    return limited_input;
}
