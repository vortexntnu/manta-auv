#include <reference_filter_dp/reference_filter_ros.hpp>

ReferenceFilterNode::ReferenceFilterNode()
    : Node("reference_filter_node") {
    time_step_ = std::chrono::milliseconds(10);

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->declare_parameter<std::string>("reference_filter_topic", "/reference_topic");
    std::string reference_filter_topic = this->get_parameter("reference_filter_topic").as_string();
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>("/dp/reference", 10);
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(reference_filter_topic, qos_sensor_data, std::bind(&ReferenceFilterNode::reference_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/nucleus/odom", 10, std::bind(&ReferenceFilterNode::state_callback, this, std::placeholders::_1));
    
    set_refererence_filter();

    action_server_ = rclcpp_action::create_server<vortex_msgs::action::ReferenceFilterWaypoint>(
        this,
        "reference_filter",
        std::bind(&ReferenceFilterNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ReferenceFilterNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ReferenceFilterNode::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    x_ = Vector18d::Zero();
    current_state_ = nav_msgs::msg::Odometry();
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta", {0.707, 0.707, 0.707, 0.707, 0.707, 0.707});
    this->declare_parameter<std::vector<double>>("omega", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    
    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    const Vector6d zeta_eigen = Eigen::Map<Vector6d>(zeta.data());
    const Vector6d omega_eigen = Eigen::Map<Vector6d>(omega.data());

    reference_filter_.set_delta(zeta_eigen);
    reference_filter_.set_omega(omega_eigen);

    reference_filter_.calculate_Ad();
    reference_filter_.calculate_Bd();

}


void ReferenceFilterNode::reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    tf2::Quaternion q;
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    r_ << x, y, z, roll, pitch, yaw;
}

void ReferenceFilterNode::state_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_ = *msg;
}

rclcpp_action::GoalResponse ReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal) {
    (void)uuid;
    (void)goal;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Aborting current goal and accepting new goal");
                preempted_goal_id_ = goal_handle_->get_goal_id();
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle)
{
    execute(goal_handle);
}
    

void ReferenceFilterNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    x_ = Vector18d::Zero();
    x_(0) = current_state_.pose.pose.position.x;
    x_(1) = current_state_.pose.pose.position.y;
    x_(2) = current_state_.pose.pose.position.z;

    tf2::Quaternion q;
    q.setX(current_state_.pose.pose.orientation.x);
    q.setY(current_state_.pose.pose.orientation.y);
    q.setZ(current_state_.pose.pose.orientation.z);
    q.setW(current_state_.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    x_(3) = ssa(roll);
    x_(4) = ssa(pitch);
    x_(5) = ssa(yaw);

    Vector6d eta;
    eta << current_state_.pose.pose.position.x, current_state_.pose.pose.position.y, current_state_.pose.pose.position.z, roll, pitch, yaw;
    Matrix6d J = calculate_J(eta);
    Vector6d nu;
    nu << current_state_.twist.twist.linear.x, current_state_.twist.twist.linear.y, current_state_.twist.twist.linear.z, current_state_.twist.twist.angular.x, current_state_.twist.twist.angular.y, current_state_.twist.twist.angular.z;
    Vector6d eta_dot = J * nu;

    x_(6) = eta_dot(0);
    x_(7) = eta_dot(1);
    x_(8) = eta_dot(2);
    x_(9) = eta_dot(3);
    x_(10) = eta_dot(4);
    x_(11) = eta_dot(5);

    const geometry_msgs::msg::PoseStamped goal = goal_handle->get_goal()->goal;

    double x = goal.pose.position.x;
    double y = goal.pose.position.y;
    double z = goal.pose.position.z;

    tf2::Quaternion q_goal;
    q_goal.setX(goal.pose.orientation.x);
    q_goal.setY(goal.pose.orientation.y);
    q_goal.setZ(goal.pose.orientation.z);
    q_goal.setW(goal.pose.orientation.w);

    tf2::Matrix3x3 m_goal(q_goal);
    double roll_goal, pitch_goal, yaw_goal;
    m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);

    r_ << x, y, z, roll_goal, pitch_goal, yaw_goal;

    auto feedback = std::make_shared<vortex_msgs::action::ReferenceFilterWaypoint::Feedback>();
    auto result = std::make_shared<vortex_msgs::action::ReferenceFilterWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                result->result = feedback->feedback;
                goal_handle->abort(result);
                return;
            }
        }
        if (goal_handle->is_canceling()) {
            result->result = feedback->feedback;
            goal_handle->canceled(result);
            return;
        }
        Vector18d x_dot = reference_filter_.calculate_x_dot(x_, r_);
        x_ += x_dot * time_step_.count() / 1000.0;

        vortex_msgs::msg::ReferenceFilter feedback_msg;
        feedback_msg.x = x_(0);
        feedback_msg.y = x_(1);
        feedback_msg.z = x_(2);
        feedback_msg.roll = ssa(x_(3));
        feedback_msg.pitch = ssa(x_(4));
        feedback_msg.yaw = ssa(x_(5));
        feedback_msg.x_dot = x_(6);
        feedback_msg.y_dot = x_(7);
        feedback_msg.z_dot = x_(8);
        feedback_msg.roll_dot = x_(9);
        feedback_msg.pitch_dot = x_(10);
        feedback_msg.yaw_dot = x_(11);

        feedback->feedback = feedback_msg;

        // Publish feedback using the correct action feedback message
        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(feedback_msg);

        if ((x_.segment<6>(0) - r_).norm() < 1e-5) {
            RCLCPP_INFO(this->get_logger(), "Goal achieved");

            // Populate result
            result->result = feedback_msg;

            goal_handle->succeed(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        loop_rate.sleep();
    }

}