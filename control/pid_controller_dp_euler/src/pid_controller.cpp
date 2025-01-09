#include <pid_controller_dp_euler/pid_controller.hpp>
#include <pid_controller_dp_euler/pid_controller_utils.hpp>

PIDController::PIDController()
    : Kp_(Matrix6d::Identity()),
      Ki_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      integral_(Vector6d::Zero()),
      dt_(0.01) {}

Vector6d PIDController::calculate_tau(const Eta& eta,
                                      const Eta& eta_d,
                                      const Nu& nu,
                                      const Eta& eta_dot_d) {
    Eta error = apply_ssa(eta - eta_d);

    Matrix6d j = calculate_j(eta);

    Matrix6d j_inv = j.inverse();

    Vector6d nu_d = j_inv * eta_dot_d.to_vector();

    Vector6d error_nu = nu.to_vector() - nu_d;

    Vector6d tau = -(Kp_ * j_inv * error.to_vector() + Ki_ * j_inv * integral_ +
                     Kd_ * error_nu);

    tau = limit_input(tau);

    integral_ = anti_windup(dt_, error.to_vector(), integral_);

    return tau;
}

void PIDController::setKp(const Matrix6d& Kp) {
    this->Kp_ = Kp;
}

void PIDController::setKi(const Matrix6d& Ki) {
    this->Ki_ = Ki;
}

void PIDController::setKd(const Matrix6d& Kd) {
    this->Kd_ = Kd;
}

void PIDController::setTimeStep(double dt) {
    this->dt_ = dt;
}
