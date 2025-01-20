#include <iostream>
#include <pid_controller_dp/pid_controller_ros.hpp>
#include <variant>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

PIDControllerNode::PIDControllerNode() : Node("pid_controller_node") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    time_step_ = std::chrono::milliseconds(10);

    this->declare_parameter("dp_reference_topic", "/dp/reference");
    this->declare_parameter("control_topic", "/thrust/wrench_input");
    this->declare_parameter("software_kill_switch_topic",
                            "/softwareKillSwitch");
    this->declare_parameter("software_operation_mode_topic",
                            "/softwareOperationMode");

    this->declare_parameter("pose_topic", "/dvl/pose");
    this->declare_parameter("twist_topic", "/dvl/twist");

    std::string dp_reference_topic =
        this->get_parameter("dp_reference_topic").as_string();
    std::string control_topic =
        this->get_parameter("control_topic").as_string();
    std::string software_kill_switch_topic =
        this->get_parameter("software_kill_switch_topic").as_string();
    std::string software_operation_mode_topic =
        this->get_parameter("software_operation_mode_topic").as_string();

    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    std::string twist_topic = this->get_parameter("twist_topic").as_string();

    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, 10,
        std::bind(&PIDControllerNode::killswitch_callback, this,
                  std::placeholders::_1));
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, 10,
        std::bind(&PIDControllerNode::software_mode_callback, this,
                  std::placeholders::_1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::pose_callback, this,
                  std::placeholders::_1));

    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::twist_callback, this,
                  std::placeholders::_1));

    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&PIDControllerNode::guidance_callback, this,
                      std::placeholders::_1));
    kp_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/kp", qos_sensor_data,
        std::bind(&PIDControllerNode::kp_callback, this,
                  std::placeholders::_1));
    ki_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/ki", qos_sensor_data,
        std::bind(&PIDControllerNode::ki_callback, this,
                  std::placeholders::_1));
    kd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/kd", qos_sensor_data,
        std::bind(&PIDControllerNode::kd_callback, this,
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

void PIDControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    eta_ = eta_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    nu_ = nu_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    types::Vector6d tau =
        pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);

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

void PIDControllerNode::kp_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Kp = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKp(Kp);
}

void PIDControllerNode::ki_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Ki = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKi(Ki);
}

void PIDControllerNode::kd_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Kd = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKd(Kd);
}

void PIDControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.pos << msg->x, msg->y, msg->z;

    double roll = msg->roll;
    double pitch = msg->pitch;
    double yaw = msg->yaw;

    eta_d_.ori = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}
