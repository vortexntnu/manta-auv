/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct Eta {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Eta operator-(const Eta& other) const {
        Eta eta;
        eta.x = x - other.x;
        eta.y = y - other.y;
        eta.z = z - other.z;
        eta.roll = roll - other.roll;
        eta.pitch = pitch - other.pitch;
        eta.yaw = yaw - other.yaw;
        return eta;
    }

    Vector6d to_vector() const {
        Vector6d eta;
        eta << x, y, z, roll, pitch, yaw;
        return eta;
    }

    // @brief Make the rotation matrix according to eq. 2.31 in Fossen, 2021
    Matrix3d as_rotation_matrix() const {
        double cphi = cos(roll);
        double sphi = sin(roll);
        double ctheta = cos(pitch);
        double stheta = sin(pitch);
        double cpsi = cos(yaw);
        double spsi = sin(yaw);

        double r11 = cpsi * ctheta;
        double r12 = cpsi * stheta * sphi - spsi * cphi;
        double r13 = cpsi * stheta * cphi + spsi * sphi;
        double r21 = spsi * ctheta;
        double r22 = spsi * stheta * sphi + cpsi * cphi;
        double r23 = spsi * stheta * cphi - cpsi * sphi;
        double r31 = -stheta;
        double r32 = ctheta * sphi;
        double r33 = ctheta * cphi;

        Matrix3d rotation_matrix;
        rotation_matrix << r11, r12, r13, r21, r22, r23, r31, r32, r33;
        return rotation_matrix;
    }

    // @brief Make the transformation matrix according to eq. 2.41 in Fossen,
    // 2021
    Matrix3d as_transformation_matrix() const {
        double cphi = cos(roll);
        double sphi = sin(roll);
        double ctheta = cos(pitch);
        double stheta = sin(pitch);

        if (ctheta == 0) {
            throw std::runtime_error(
                "Division by zero in transformation matrix.");
        }

        double t11 = 1;
        double t12 = sphi * stheta / ctheta;
        double t13 = cphi * stheta / ctheta;
        double t21 = 0;
        double t22 = cphi;
        double t23 = -sphi;
        double t31 = 0;
        double t32 = sphi / ctheta;
        double t33 = cphi / ctheta;

        Matrix3d transformation_matrix;

        transformation_matrix << t11, t12, t13, t21, t22, t23, t31, t32, t33;

        return transformation_matrix;
    }
};

struct Nu {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    double p = 0.0;
    double q = 0.0;
    double r = 0.0;

    Nu operator-(const Nu& other) const {
        Nu nu;
        nu.u = u - other.u;
        nu.v = v - other.v;
        nu.w = w - other.w;
        nu.p = p - other.p;
        nu.q = q - other.q;
        nu.r = r - other.r;
        return nu;
    }

    Vector6d to_vector() const {
        Vector6d nu;
        nu << u, v, w, p, q, r;
        return nu;
    }
};

#endif
