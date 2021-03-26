#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "MiniPID.h"

// These typdefs are lacking from the default eigen namespace
namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

const std::string DEFAULT_ODOM_TOPIC = "/odometry/filtered";
const std::string DEFAULT_THRUST_TOPIC = "/thrust/desired";
const std::string DEFAULT_VELOCITY_TOPIC = "/controller/desired_velocity";

class VelocityController
{
public:
  VelocityController(ros::NodeHandle ros_node);
  void odometryCallback(const nav_msgs::Odometry& odom_msg);
  void controlLawCallback(const geometry_msgs::Twist& twist_msg);
  template<typename T>
  void getParam(std::string name, T &variable);
  template<typename T>
  void getParam(std::string name, T &variable, T default_value);
  Eigen::Vector6d restoringForces();

private:
  ros::NodeHandle ros_node;
  std::string odometry_topic;
  std::string thrust_topic;
  std::string desired_velocity_topic;
  double drone_weight;
  float drone_bouyancy; 
  Eigen::Vector3d center_of_gravity;
  Eigen::Vector3d center_of_bouyancy;
  ros::Publisher thrust_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber vel_sub;
  Eigen::Vector6d velocity;
  Eigen::Quaterniond orientation;
};

#endif