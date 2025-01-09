#include <reference_filter_dp/reference_filter_utils.hpp>

Matrix3d calculate_R(const Vector6d& eta) {
    double phi = eta(3);
    double theta = eta(4);
    double psi = eta(5);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double cpsi = cos(psi);
    double spsi = sin(psi);

    Matrix3d rotation_matrix;

    rotation_matrix << cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi,
        cpsi * stheta * cphi + spsi * sphi, spsi * ctheta,
        spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi,
        -stheta, ctheta * sphi, ctheta * cphi;

    return rotation_matrix;
}

Matrix3d calculate_T(const Vector6d& eta) {
    Matrix3d transformation_matrix;
    double phi = eta(3);
    double theta = eta(4);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);

    // Manually added the transformation matrix from Fossen 2021 p.29 eq: 2.41
    transformation_matrix << 1, sphi * stheta / ctheta, cphi * stheta / ctheta,
        0, cphi, -sphi, 0, sphi / ctheta, cphi / ctheta;

    return transformation_matrix;
}

Matrix6d calculate_J(const Vector6d& eta) {
    Matrix3d R = calculate_R(eta);
    Matrix3d T = calculate_T(eta);

    Matrix6d J = Matrix6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(3, 3) = T;

    return J;
}

double ssa(double angle) {
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
}
