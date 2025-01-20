#ifndef REFERENCE_FILTER_ROS_HPP
#define REFERENCE_FILTER_ROS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <reference_filter_dp/reference_filter.hpp>
#include <reference_filter_dp/reference_filter_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode();

   private:
    // @brief Initializes the reference filter with ROS parameters.
    void set_refererence_filter();

    // @brief Callback for the reference topic
    // @param msg The reference message
    void reference_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // @brief Callback for the state topic
    // @param msg The state message
    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal);

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    Vector18d fill_reference_state();

    Vector6d fill_reference_goal(const geometry_msgs::msg::PoseStamped& goal);

    vortex_msgs::msg::ReferenceFilter fill_reference_msg();

    rclcpp_action::Server<
        vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr action_server_;

    ReferenceFilter reference_filter_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;

    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_;

    nav_msgs::msg::Odometry current_state_;

    // x is [eta, eta_dot, eta_dot_dot] (ref. page 337 in Fossen, 2021
    // nu and eta are 6 degrees of freedom (position and orientation in 3D
    // space)
    Vector18d x_;

    // The reference signal vector with 6 degrees of freedom [eta]
    Vector6d r_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>>
        goal_handle_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

#endif
