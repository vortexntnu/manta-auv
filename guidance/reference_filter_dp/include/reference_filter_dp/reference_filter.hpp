#ifndef REFERENCE_FILTER_HPP
#define REFERENCE_FILTER_HPP

#include "reference_filter_dp/eigen_typedefs.hpp"

class ReferenceFilter {
   public:
    explicit ReferenceFilter();

    // @brief Calculate the state derivative
    // @param x The state vector 18x1
    // @param r The reference vector 6x1
    // @return The state derivative 18x1
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.11
    Vector18d calculate_x_dot(const Vector18d& x, const Vector6d& r);

    // @brief Calculate the state transition matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.12
    void calculate_Ad();

    // @brief Calculate the input matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.12
    void calculate_Bd();

    // @brief Set the omega matrix
    // @param omega The omega matrix 6x1 vector
    void set_omega(const Vector6d& omega);

    // @brief Set the delta matrix
    // @param zeta The delta matrix 6x1 vector
    void set_delta(const Vector6d& zeta);

   private:
    Matrix18d Ad_;
    Matrix18x6d Bd_;
    Matrix6d Omega_;
    Matrix6d Delta_;
    Matrix6d identity_matrix_;
};

#endif
