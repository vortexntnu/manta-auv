#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

DPAdaptBacksController::DPAdaptBacksController()
    : K1_(dp_types::Matrix6d::Identity()),
      K2_(dp_types::Matrix6d::Identity()),
      r_b_bg_(dp_types::Vector3d::Zero()),
      adapt_gain_(dp_types::Matrix12d::Identity()),
      d_gain_(dp_types::Matrix6d::Zero()),
      adap_param_(dp_types::Vector12d::Zero()),
      d_est_(dp_types::Vector6d::Zero()),
      I_b_(dp_types::Matrix3d::Identity()),
      M_(dp_types::Matrix6d::Identity()),
      m_(0),
      dt_(0.01) {}

dp_types::Vector6d DPAdaptBacksController::calculate_tau(
    const dp_types::Eta& eta,
    const dp_types::Eta& eta_d,
    const dp_types::Nu& nu) {
    dp_types::Eta error = error_eta(eta, eta_d);

    dp_types::Matrix6d C = calculate_C(m_, r_b_bg_, nu, I_b_);

    dp_types::Matrix6d J = calculate_J(error);

    dp_types::Matrix6d J_inv = calculate_J_sudo_inv(error);

    dp_types::Matrix6d J_dot = calculate_J_dot(error, nu);

    dp_types::Vector6d alpha = -J_inv * K1_ * error.as_vector();

    dp_types::Vector6d z_1 = error.as_vector();

    dp_types::Vector6d z_2 = nu.as_vector() - alpha;

    dp_types::Vector6d alpha_dot =
        ((J_inv * J_dot * J_inv) * K1_ * z_1) - (J_inv * K1_ * nu.as_vector());

    dp_types::Matrix6x12d Y_v = calculate_Y_v(nu);

    dp_types::Vector12d adapt_param_dot = adapt_gain_ * Y_v.transpose() * z_2;

    dp_types::Vector6d d_est_dot = d_gain_ * z_2;

    dp_types::Vector6d F_est = Y_v * adap_param_;

    dp_types::Vector6d tau = (M_ * alpha_dot) + (C * nu.as_vector()) -
                             (J.transpose() * z_1) - F_est - d_est_;

    adap_param_ = adap_param_ + adapt_param_dot * dt_;

    d_est_ = d_est_ + d_est_dot * dt_;

    std::cout << "M: " << m_ << std::endl;

    return error.as_vector();
}

void DPAdaptBacksController::setK1(const dp_types::Vector6d& K1) {
    this->K1_ = K1.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setK2(const dp_types::Vector6d& K2) {
    this->K2_ = K2.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setrbg(const dp_types::Vector3d& r_b_bg) {
    this->r_b_bg_ = r_b_bg;
}

void DPAdaptBacksController::setAdapParam(
    const dp_types::Vector12d& adapt_gain) {
    this->adapt_gain_ = adapt_gain.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setDGain(const dp_types::Vector6d& d_gain) {
    this->d_gain_ = d_gain.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setm(double m) {
    this->m_ = m;
}

void DPAdaptBacksController::setInertiaMatrix(const dp_types::Vector3d& I_b) {
    this->I_b_ = I_b.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setMassInertiaMatrix(const dp_types::Matrix6d& M) {
    this->M_ = M;
}

void DPAdaptBacksController::setTimeStep(double dt) {
    this->dt_ = dt;
}
