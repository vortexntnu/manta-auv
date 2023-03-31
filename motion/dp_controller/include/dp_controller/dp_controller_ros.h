/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief A ROS wrapper layer for the quaternion PID controller
 *
 */
#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <map>
#include <math.h>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include "eigen_typedefs.h"

#include "dp_controller/quaternion_dp_controller.h"

#include <dp_controller/DpControllerConfig.h> //autogenerated from Controller.cfg
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/server.h>
// template <typename T>

/**
 * @brief the Controller class
 *
 * This class serves as a wrapper for the lower-level controller implementation
 * @see quaternion_pd_controller.h
 *
 */
class Controller {
private:
  /**
   * @brief Desired pose in quaternions.
   */
 
  Eigen::Vector3d m_eta_d_pos;
  Eigen::Quaterniond m_eta_d_ori;
  Eigen::Vector6d m_nu_d;

  double m_rate;
   std::vector<double> m_acceptance_margins_vec;

  ros::NodeHandle m_nh; /** Nodehandle          */

  ros::Subscriber m_odometry_sub; /** Odometry subscriber    */

  ros::Publisher m_wrench_pub; /** Wrench publisher    */

  //---------Debug -------
  bool m_debug = true;
  ros::Publisher m_dp_P_debug_pub;
  ros::Publisher m_dp_I_debug_pub;
  ros::Publisher m_dp_D_debug_pub;
  ros::Publisher m_dp_g_debug_pub;
  //----------------------

  std::vector<int> m_enable_PID;

  ros::Subscriber m_desiredpoint_sub;  /* Subscriber for listening to (the
                                          guidance node ....)      */
  ros::Publisher m_referencepoint_pub; /* Publisher for the DP-controller */

  void cfgCallback(dp_controller::DpControllerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<dp_controller::DpControllerConfig> m_cfg_server;

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d m_position;       /** Current position      */
  Eigen::Quaterniond m_orientation; /** Current orientation   */
  Eigen::Vector6d m_velocity;       /** Current velocity      */

  QuaternionPIDController m_controller;

  template <typename T>
  void getParameters(std::string param_name, T &param_variable);

public:
  /**
   * @brief Controller class constructor
   *
   * @param nh ROS nodehandle
   */
  explicit Controller(ros::NodeHandle nh);

  /**
   * @brief Callback for the odometry subscriber
   *
   * @param msg   A nav_msg::Odometry message containing state data about the
   * AUV.
   */

  void odometryCallback(const nav_msgs::Odometry &msg);
  void desiredPointCallback(const nav_msgs::Odometry &desired_msg);
  Eigen::Vector3d quaterniondToEuler(Eigen::Quaterniond q);
  void spin();
};

#endif // VORTEX_CONTROLLER_CONTROLLER_ROS_H
