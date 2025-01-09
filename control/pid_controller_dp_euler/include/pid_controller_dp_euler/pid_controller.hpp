#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <pid_controller_dp_euler/typedefs.hpp>

class PIDController {
   public:
    explicit PIDController();

    Vector6d calculate_tau(const Eta& eta,
                           const Eta& eta_d,
                           const Nu& nu,
                           const Eta& eta_dot_d);

    void setKp(const Matrix6d& Kp);

    void setKi(const Matrix6d& Ki);

    void setKd(const Matrix6d& Kd);

    void setTimeStep(double dt);

   private:
    Matrix6d Kp_;
    Matrix6d Ki_;
    Matrix6d Kd_;
    Vector6d integral_;
    double dt_;
};

#endif
