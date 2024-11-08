#ifndef PID_CONTROLLER_ROS_HPP
#define PID_CONTROLLER_ROS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include "pid_controller_dp/pid_controller.hpp"

// @brief Class for the PID controller node
class PIDControllerNode : public rclcpp::Node {
   public:
    explicit PIDControllerNode();

   private:
    // @brief Callback function for the odometry topic
    // @param msg: Odometry message containing the vehicle pose and velocity
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // @brief Callback function for the proportional gain matrix
    void publish_tau();

    // @brief Set the PID controller parameters
    void set_pid_params();

    // @brief Callback function for the proportional gain matrix
    // @param msg: Float64MultiArray message containing the proportional gain
    // matrix
    void kp_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // @brief Callback function for the integral gain matrix
    // @param msg: Float64MultiArray message containing the integral gain matrix
    void ki_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // @brief Callback function for the derivative gain matrix
    // @param msg: Float64MultiArray message containing the derivative gain
    // matrix
    void kd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // @brief Callback function for the guidance topic
    // @param msg: ReferenceFilter message containing the desired vehicle pose
    // and velocity
    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilter::SharedPtr msg);

    PIDController pid_controller_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        guidance_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kp_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ki_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kd_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr tau_pub_;

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    Eigen::Vector7d eta_;

    Eigen::Vector7d eta_d_;

    Eigen::Vector6d nu_;

    Eigen::Vector7d eta_dot_d_;
};

#endif
