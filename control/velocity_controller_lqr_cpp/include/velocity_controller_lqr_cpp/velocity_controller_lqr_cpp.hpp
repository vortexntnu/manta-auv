#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vortex_msgs/msg/los_guidance.hpp>

// Global variables
Eigen::VectorXd state_values(6); Eigen::VectorXd guidance_values(3); // State and guidance variables
Eigen::VectorXd state_error(6); Eigen::VectorXd integral_error(3); // Error Variables

Eigen::MatrixXd Q(6,6); Eigen::MatrixXd R(3,3); // Cost matrices
Eigen::MatrixXd M(3,3); Eigen::MatrixXd M_inv(3,3); Eigen::MatrixXd C(3,3); // Matrices for the state space model
Eigen::MatrixXd A_3x3(3,3); Eigen::MatrixXd A(6,6); Eigen::MatrixXd B(6,3); Eigen::MatrixXd K(3,6);
Eigen::MatrixXd A_d(6,6); Eigen::MatrixXd B_d(6,3); Eigen::MatrixXd K_d(3,6); // Discrete state space model matrices
Eigen::VectorXd u(3);

bool abu = false;
double max_force = 97.5;

// Function to convert quaternion to Euler angles (roll, pitch, yaw)
Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z) {
    Eigen::Vector3d angle_vector;

    // Roll (x-axis rotation)
    angle_vector(0) = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    
    // Pitch (y-axis rotation)
    angle_vector(1) = std::asin(2.0 * (w * y - z * x));
    
    // Yaw (z-axis rotation)
    angle_vector(2) = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    
    return  angle_vector; // Return the Euler angles as a Vector3d (roll, pitch, yaw)
}

// Function to convert Euler angles (roll, pitch, yaw) to quaternion
double ssa(double angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void calculate_coriolis_matrix(double pitch_rate, double yaw_rate, double sway, double heave, Eigen::MatrixXd &C) {
    C << 0.2, -30 * sway * 0.01, -30 * heave * 0.01,
        30 * sway * 0.01, 0, 1.629 * pitch_rate,
        30 * heave * 0.01, 1.769 * yaw_rate, 0;
}

double saturate(double value, double max_force, bool &saturation) {
    if (abs(value) > max_force) {
        saturation = true;
        return max_force * value / abs(value);
    } else {
        saturation = false;
        return value;
    }
}

void saturate_integral(double &integral_sum, double error, double K_i, bool windup) {
    if (windup) {
        integral_sum += 0;
    } else {
        integral_sum += K_i * error;
    }
}

void discretizeSystem(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double T_s, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d) {
    // A_d = exp(A * T_s)
    A_d = (A * T_s).exp(); // Matrix exponential using Eigen

    // B_d = Integral of exp(A * tau) * B from 0 to T_s
    Eigen::MatrixXd integral = (A_d - Eigen::MatrixXd::Identity(A.rows(), A.cols())) * A.inverse();
    B_d = integral * B;
}

// Function to solve the DARE using the Schur decomposition
Eigen::MatrixXd solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R) {
    int n = A.rows();     // State dimension
    int m = B.cols();     // Input dimension

    // Formulate the Hamiltonian matrix
    Eigen::MatrixXd Z(2 * n, 2 * n);
    Z.topLeftCorner(n, n) = A + B * R.inverse() * B.transpose() * Q;
    Z.topRightCorner(n, n) = -B * R.inverse() * B.transpose();
    Z.bottomLeftCorner(n, n) = -Q;
    Z.bottomRightCorner(n, n) = A.transpose();

    // Perform the Schur decomposition
    Eigen::RealSchur<Eigen::MatrixXd> schur(Z);
    Eigen::MatrixXd U = schur.matrixU();
    Eigen::MatrixXd T = schur.matrixT();

    // Extract the stable subspace
    Eigen::MatrixXd U11 = U.topLeftCorner(n, n);
    Eigen::MatrixXd U21 = U.bottomLeftCorner(n, n);

    // Calculate the stabilizing solution to the DARE
    Eigen::MatrixXd P = U21 * U11.inverse();
    return P;
}

Eigen::MatrixXd computeLQRGain(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R) {
    Eigen::MatrixXd P = solveDARE(A, B, Q, R);
    Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
    return K;
}

// Class definition
class VelocityLQRNode : public rclcpp::Node 
{
public:
    VelocityLQRNode() : Node("velocity_controller_lqr") {
        // Initialize the publishers
        pub_thrust = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 15);
        pub_foxglove = this->create_publisher<std_msgs::msg::Float32MultiArray>("/velocity/states", 20);

        // Initialize timers
        update_state_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&VelocityLQRNode::update_state_timer_callback, this));
        control_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VelocityLQRNode::control_timer_callback, this));
        foxglove_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&VelocityLQRNode::foxglove_timer_callback, this));
        
        // Initialize the subscribers
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/nucleus/odom", 40, std::bind(&VelocityLQRNode::odom_callback, this, std::placeholders::_1));
        sub_guidance = this->create_subscription<vortex_msgs::msg::LOSGuidance>("/guidance/los", 40, std::bind(&VelocityLQRNode::guidance_callback, this, std::placeholders::_1));

        this->declare_parameter("P_surge_", 2.5);
        this->declare_parameter("P_pitch_", 10.0);
        this->declare_parameter("P_yaw_",  7.5);
        this->declare_parameter("I_surge_", 0.005);
        this->declare_parameter("I_pitch_", 0.01);
        this->declare_parameter("I_yaw_", 0.005);

        P_surge_ = this->get_parameter("P_surge_").as_double();
        P_pitch_ = this->get_parameter("P_pitch_").as_double();
        P_yaw_ = this->get_parameter("P_yaw_").as_double();
        I_surge_ = this->get_parameter("I_surge_").as_double();
        I_pitch_ = this->get_parameter("I_pitch_").as_double();
        I_yaw_ = this->get_parameter("I_yaw_").as_double();

        guidance_values << 0.3, -M_PI/4, M_PI/2; // Initial guidance values

        // Integral sums start at 0
        surge_integral_sum_ = 0;
        pitch_integral_sum_ = 0;
        yaw_integral_sum_ = 0;

        // Cost matrices
        Q << P_surge_ , 0.0 , 0.0 , 0.0 , 0.0 , 0.0,
            0.0 , P_pitch_ , 0.0 , 0.0 , 0.0 , 0.0,
            0.0 , 0.0 , P_yaw_ , 0.0 , 0.0 , 0.0,
            0.0 , 0.0 , 0.0 , 1 , 0.0 , 0.0,
            0.0 , 0.0 , 0.0 , 0.0 , 1 , 0.0,
            0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1;

        this->declare_parameter("R_surge_", 1.5);
        this->declare_parameter("R_pitch_", 1.0);
        this->declare_parameter("R_yaw_", 1.0);

        R_surge_ = this->get_parameter("R_surge_").as_double();
        R_pitch_ = this->get_parameter("R_pitch_").as_double();
        R_yaw_= this->get_parameter("R_yaw_").as_double();
        
        R << R_surge_ , 0.0 , 0.0,
            0.0 , R_pitch_ , 0.0,
            0.0 , 0.0 , R_yaw_;

        C.setZero(); A_3x3.setZero(); B.setZero(); A.setZero(); K.setZero();

        M << 30.0 , 0.6 , 0.0,
            0.6, 1.629, 0.0,
            0.0  , 0.0    , 1.769;

        M_inv = M.inverse();
        RCLCPP_INFO(this->get_logger(), "Velocity Controller LQR Node has been initialized");
    }

private:

    bool surge_windup = false; bool pitch_windup = false; bool yaw_windup = false; //windup flags
    double P_surge_; double P_pitch_; double P_yaw_;
    double surge_integral_sum_; double pitch_integral_sum_; double yaw_integral_sum_;
    double I_surge_; double I_pitch_; double I_yaw_;
    double R_surge_; double R_pitch_; double R_yaw_;


    // Callback function for the odometry subscriber
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

        Eigen::Vector3d euler_angles = quaternionToEuler(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        state_values(0) = msg->twist.twist.linear.x; // Surge
        state_values(1) = euler_angles(1); // Pitch
        state_values(2) = euler_angles(2); // Yaw

        calculate_coriolis_matrix(msg->twist.twist.angular.y, msg->twist.twist.angular.z, msg->twist.twist.linear.y, msg->twist.twist.linear.z, C);
    }

    void guidance_callback(const vortex_msgs::msg::LOSGuidance::SharedPtr msg){
        guidance_values(0) = msg->surge; // Surge
        guidance_values(1) = msg->pitch; // Pitch
        guidance_values(2) = msg->yaw; // Yaw
    }

    void update_state_timer_callback(){
        state_error.head<3>() << (guidance_values(0) - state_values(0)), ssa(guidance_values(1) - state_values(1)), ssa(guidance_values(2) - state_values(2));

        saturate_integral(surge_integral_sum_, state_error(0), I_surge_, surge_windup);
        saturate_integral(pitch_integral_sum_, state_error(1), I_pitch_, pitch_windup);
        saturate_integral(yaw_integral_sum_, state_error(2), I_yaw_, yaw_windup);

        integral_error << surge_integral_sum_, pitch_integral_sum_, yaw_integral_sum_;
        state_error.tail<3>() << -integral_error;

        RCLCPP_INFO(this->get_logger(), "\n--------------------------------------------------------------------------\n");
        std::cout << "State error: " << state_error << std::endl;
    }

    void foxglove_timer_callback(){
        auto msg = std_msgs::msg::Float32MultiArray();

        // 0. Surge, 1. Pitch, 2. Yaw, 3. Surge Guidance, 4. Pitch Guidance, 5. Yaw Guidance, 6. Surge Error, 7. Pitch Error
        std::vector<float> foxglove_values = {
        static_cast<float>(state_values(0)),
        static_cast<float>(state_values(1)),
        static_cast<float>(state_values(2)),
        static_cast<float>(guidance_values(0)),
        static_cast<float>(guidance_values(1)),
        static_cast<float>(guidance_values(2)),
        abs(static_cast<float>(state_error(1))),
        abs(static_cast<float>(state_error(2)))
    };
        msg.data = foxglove_values;
        pub_foxglove->publish(msg);
    }

    void control_timer_callback(){
        auto msg = geometry_msgs::msg::Wrench();
        // Calculate the state space model

        A_3x3 = -1 * M_inv * C;
        
        A << A_3x3, Eigen::MatrixXd::Zero(3,3),
            -1 * Eigen::MatrixXd::Identity(3,3), 0.005 * Eigen::MatrixXd::Identity(3,3);
        
        B << M_inv,
             Eigen::MatrixXd::Zero(3,3);

        // // Discretize the state space model
        discretizeSystem(A, B, 0.1, A_d, B_d);
        // Solve DARE to get P matrix

        Eigen::MatrixXd K_d = computeLQRGain(A_d, B_d, Q, R);
        u = -1 * (K_d * state_error);

        msg.force.x = saturate(u(0), max_force, surge_windup);
        msg.torque.y = saturate(u(1), max_force, pitch_windup);
        msg.torque.z = saturate(u(2), max_force, yaw_windup);

        RCLCPP_INFO(this->get_logger(), "Thrust: %f\n, Pitch: %f\n, Yaw: %f\n\n\n-----------------------------------------------------------------", u(0), u(1), u(2));
        pub_thrust->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_thrust;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_foxglove;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::TimerBase::SharedPtr update_state_timer;
    rclcpp::TimerBase::SharedPtr foxglove_timer;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<vortex_msgs::msg::LOSGuidance>::SharedPtr sub_guidance;
};