#include <pid_controller_dp_euler/pid_controller_ros.hpp>
#include <pid_controller_dp_euler/pid_controller_utils.hpp>
#include <string>

PIDControllerNode::PIDControllerNode() : Node("pid_controller_euler_node") {
    time_step_ = std::chrono::milliseconds(10);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    this->declare_parameter("odom_topic", "/orca/odom");
    this->declare_parameter("reference_topic", "/dp/reference");
    this->declare_parameter("control_topic", "/thrust/wrench_input");
    this->declare_parameter("software_kill_switch_topic",
                            "/softwareKillSwitch");
    this->declare_parameter("software_operation_mode_topic",
                            "/softwareOperationMode");

    std::string nucleus_odom_topic =
        this->get_parameter("odom_topic").as_string();
    std::string dp_reference_topic =
        this->get_parameter("reference_topic").as_string();
    std::string control_topic =
        this->get_parameter("control_topic").as_string();
    std::string software_kill_switch_topic =
        this->get_parameter("software_kill_switch_topic").as_string();
    std::string software_operation_mode_topic =
        this->get_parameter("software_operation_mode_topic").as_string();

    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, 10,
        std::bind(&PIDControllerNode::killswitch_callback, this,
                  std::placeholders::_1));
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, 10,
        std::bind(&PIDControllerNode::software_mode_callback, this,
                  std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        nucleus_odom_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::odometry_callback, this,
                  std::placeholders::_1));
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, 10,
            std::bind(&PIDControllerNode::guidance_callback, this,
                      std::placeholders::_1));
    tau_pub_ =
        this->create_publisher<geometry_msgs::msg::Wrench>(control_topic, 10);
    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    set_pid_params();
}

void PIDControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
}

void PIDControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
}

void PIDControllerNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    eta_.x = msg->pose.pose.position.x;
    eta_.y = msg->pose.pose.position.y;
    eta_.z = msg->pose.pose.position.z;

    tf2::Quaternion quat;
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    quat.setW(msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    eta_.roll = roll;
    eta_.pitch = pitch;
    eta_.yaw = yaw;

    nu_.u = msg->twist.twist.linear.x;
    nu_.v = msg->twist.twist.linear.y;
    nu_.w = msg->twist.twist.linear.z;
    nu_.p = msg->twist.twist.angular.x;
    nu_.q = msg->twist.twist.angular.y;
    nu_.r = msg->twist.twist.angular.z;
}

void PIDControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    Vector6d tau = pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);

    geometry_msgs::msg::Wrench tau_msg;
    tau_msg.force.x = tau(0);
    tau_msg.force.y = tau(1);
    tau_msg.force.z = tau(2);
    tau_msg.torque.x = tau(3);
    tau_msg.torque.y = tau(4);
    tau_msg.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void PIDControllerNode::set_pid_params() {
    this->declare_parameter<std::vector<double>>(
        "Kp", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "Ki", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<std::vector<double>>(
        "Kd", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::vector<double> Kp_vec = this->get_parameter("Kp").as_double_array();
    std::vector<double> Ki_vec = this->get_parameter("Ki").as_double_array();
    std::vector<double> Kd_vec = this->get_parameter("Kd").as_double_array();

    std_msgs::msg::Float64MultiArray Kp_msg;
    std_msgs::msg::Float64MultiArray Ki_msg;
    std_msgs::msg::Float64MultiArray Kd_msg;

    Kp_msg.data = Kp_vec;
    Ki_msg.data = Ki_vec;
    Kd_msg.data = Kd_vec;

    pid_controller_.setKp(float64multiarray_to_diagonal_matrix6d(Kp_msg));
    pid_controller_.setKi(float64multiarray_to_diagonal_matrix6d(Ki_msg));
    pid_controller_.setKd(float64multiarray_to_diagonal_matrix6d(Kd_msg));
}

void PIDControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.x = msg->x;
    eta_d_.y = msg->y;
    eta_d_.z = msg->z;
    eta_d_.roll = msg->roll;
    eta_d_.pitch = msg->pitch;
    eta_d_.yaw = msg->yaw;
}
