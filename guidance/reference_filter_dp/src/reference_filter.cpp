#include <reference_filter_dp/reference_filter.hpp>

ReferenceFilter::ReferenceFilter()
    : Ad_(Matrix18d::Zero()),
      Bd_(Matrix18x6d::Zero()),
      Omega_(Matrix6d::Identity()),
      Delta_(Matrix6d::Identity()),
      identity_matrix_(Matrix6d::Identity()) {}

Vector18d ReferenceFilter::calculate_x_dot(const Vector18d& x,
                                           const Vector6d& r) {
    Vector18d x_dot = Ad_ * x + Bd_ * r;

    return x_dot;
}

void ReferenceFilter::calculate_Ad() {
    Matrix6d OmegaCubed = Omega_ * Omega_ * Omega_;
    Matrix6d OmegaSquared = Omega_ * Omega_;
    Ad_.block<6, 6>(0, 6) = identity_matrix_;
    Ad_.block<6, 6>(12, 0) = -OmegaCubed;
    Ad_.block<6, 6>(12, 6) = -(2 * Delta_ + identity_matrix_) * OmegaSquared;
    Ad_.block<6, 6>(12, 12) = -(2 * Delta_ + identity_matrix_) * Omega_;
    Ad_.block<6, 6>(6, 12) = identity_matrix_;
}

void ReferenceFilter::calculate_Bd() {
    Matrix6d OmegaCubed = Omega_ * Omega_ * Omega_;
    Bd_.block<6, 6>(12, 0) = OmegaCubed;
}

void ReferenceFilter::set_omega(const Vector6d& omega) {
    Omega_ = omega.asDiagonal();
}

void ReferenceFilter::set_delta(const Vector6d& zeta) {
    Delta_ = zeta.asDiagonal();
}
