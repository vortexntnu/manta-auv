#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import WrenchStamped, PoseStamped
from path import Path as Path1
from auv_model import AUVModel
from control_allocation import ControlAllocationSystem
from control_system import DPControlSystem
from auv_simulator import AUVSimulator
from virtual_target import VirtualTarget
from current_estimator import CurrentEstimator
from functions import inside_sphere_of_acceptence, ssa

class GuidanceAndControlNode:
    def __init__(self):
        
        '''Initialize the ROS-node'''
        rospy.init_node('guidance_and_control_system')
        while rospy.get_time() == 0:
            continue
        rospy.Subscriber('/eskf_localization/pose', Odometry, self.navigation_callback)
        # Subscribe to a waypoint action server (including a speed assignment and heading reference)
        # Subscribe to a pose hold server (to include roll and pitch the DP controller needs some work. probably not too much tho)
        self.pub = rospy.Publisher('/beluga/thruster_manager/input_stamped', WrenchStamped, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.br_eta_r = tf.TransformBroadcaster()
        self.get_pose = False

        '''Initial states'''
        self.eta_d = [0, 0, 0.5, 0, 0, 0]
        self.nu_d = [0, 0, 0, 0, 0, 0]
        self.dot_eta_c = [0, 0, 0, 0, 0, 0]
        self.mode = 'pose_hold'

        '''Initialize Fossen's equations'''
        m = rospy.get_param("/model_parameters/mass")
        r_g = rospy.get_param("/model_parameters/center_of_mass")
        r_b = rospy.get_param("/model_parameters/center_of_buoyancy")
        inertia = np.array(rospy.get_param("/model_parameters/inertia"))
        volume = rospy.get_param("/model_parameters/volume")
        M_A = np.diag(rospy.get_param("/model_parameters/M_A"))
        D = -np.diag(rospy.get_param("/model_parameters/D"))
        rho = rospy.get_param("/model_parameters/water_density")
        g = 9.81
        self.auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=rho, g=g)

        '''Initialize control allocation system'''
        thruster_positions = np.array(rospy.get_param("/thruster_parameters/positions"))
        thruster_orientations = np.array(rospy.get_param("/thruster_parameters/orientations"))
        rotor_time_constant = rospy.get_param("/thruster_parameters/first_order_time_constant")
        w = rospy.get_param("/guidance_and_control_parameters/control_forces_weights")
        u_min = rospy.get_param("/guidance_and_control_parameters/control_input_saturation_limits")[0]
        u_max = rospy.get_param("/guidance_and_control_parameters/control_input_saturation_limits")[1]
        self.control_allocation_system = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max, u_min, w)

        '''Initialize DP control system'''
        M = self.auv_model.M
        D = self.auv_model.D
        gvect = self.auv_model.gvect
        omega_b = np.array(rospy.get_param("/guidance_and_control_parameters/control_bandwidth"))
        zeta = np.array(rospy.get_param("/guidance_and_control_parameters/relative_damping_ratio"))
        self.dp_control_system = DPControlSystem(M, D, gvect, omega_b, zeta)
        
        '''Initialize reference model'''
        u_gain = rospy.get_param("/guidance_and_control_parameters/reference_model_control_input_saturation_limit_gain")
        u_min_simulator = u_min*u_gain
        u_max_simulator = u_max*u_gain
        simulator_control_allocation_system = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max_simulator, u_min_simulator, w)
        omega_b_gain = rospy.get_param("/guidance_and_control_parameters/reference_model_control_bandwidth_gain")
        self.omega_b_simulator = [x*omega_b_gain for x in omega_b]
        zeta = [1 ,1, 1, 1, 1, 1]
        simulator_control_system = DPControlSystem(M, D, gvect, self.omega_b_simulator, zeta)
        absolute_relative_velocity_limit = rospy.get_param("/guidance_and_control_parameters/absolute_relative_velocity_limit")
        self.reference_model = AUVSimulator(self.auv_model, simulator_control_allocation_system, simulator_control_system, absolute_relative_velocity_limit)
        
        '''Initialize current estimator'''
        delta_psi = "/guidance_and_control_parameters/current_estimator_heading_angle_change"
        X_u = D[0, 0]
        Y_v = D[1, 1]
        self.current_estimator = CurrentEstimator(delta_psi, X_u, Y_v)

        '''Initialize path-following controller'''
        u_gain = rospy.get_param("/guidance_and_control_parameters/virtual_target_control_input_saturation_limit_gain")
        u_min_vt= u_min*u_gain
        u_max_vt = u_max*u_gain
        self.vt_actuator_model = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max_vt, u_min_vt, w)
        self.path = Path1()
        self.waypoints = [[0, 0, 0.5], [-2, 4, 2], [-4, 4, 4], [-6, 4, 4], [-8, 3, 4], [-6, 1, 4], [-4, 2, 4], [-2, 2, 2], [0, 0, 0.5]]
        self.path.generate_G0_path(self.waypoints)
        #self.path.generate_G1_path(self.waypoints, 0.4, 0.4)  # Horizontal curvature radius
        omega_b_virtual = rospy.get_param("/guidance_and_control_parameters/virtual_target_controller_bandwidths")
        virtual_control_system = DPControlSystem(M, D, gvect, omega_b_virtual, [1, 1, 1, 1, 1, 1])
        dot_s_bounds = rospy.get_param("/guidance_and_control_parameters/virtual_target_along_track_speed_saturation_limits")
        self.path_following_controller = VirtualTarget(self.path, self.auv_model, self.vt_actuator_model, virtual_control_system, self.omega_b_simulator, dot_s_bounds=dot_s_bounds)
        
        '''Publish frequency'''
        self.publish_rate = rospy.get_param("/guidance_and_control_parameters/publish_rate")
        self.rate = rospy.Rate(self.publish_rate)

        # Publish the path for vizualization purposes
        publish_path_once(self.path)

    def navigation_callback(self, msg):
        if self.get_pose:
            self.eta = extract_from_pose(msg.pose.pose)
            self.nu = extract_from_twist(msg.twist.twist)
            self.get_pose = False
        else:
            pass

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def publish_control_forces(self):
        while not rospy.is_shutdown():
            try:
                self.get_state_estimates()
                
                # Path following mode
                if self.mode == 'path_following':

                    '''Do this only once when new path is recieved'''
                    waypoints = [[0, 0, 0.5], [-2, 4, 2]] # Extract the waypoints as shown.
                    path = Path1()
                    path.generate_G0_path(waypoints)
                    omega_b_virtual = rospy.get_param("/guidance_and_control_parameters/virtual_target_controller_bandwidths")
                    virtual_control_system = DPControlSystem(self.auv_model.M, self.auv_model.D, self.auv_model.gvect, omega_b_virtual, [1, 1, 1, 1, 1, 1])
                    dot_s_bounds = rospy.get_param("/guidance_and_control_parameters/virtual_target_along_track_speed_saturation_limits")
                    self.path_following_controller = VirtualTarget(path, self.auv_model, self.vt_actuator_model, virtual_control_system, self.omega_b_simulator, dot_s_bounds=dot_s_bounds)
                    publish_path_once(path)

                    final_wp = self.path_following_controller.path.path[-1](1)
                    final_orientation = [0, 0, 0] # Parameter server
                    pos_tol = 0.1 # Parameter server
                    heading_mode = 'path_dependent_heading' # Use either 'path_dependent_heading' or 'point_dependent_heading'
                    point = [-6, 3] # x, y coordinates
                    if inside_sphere_of_acceptence(final_wp, goal, pos_tol):
                        nu_r = np.zeros(6)
                        eta_r = np.zeros(6)
                        eta_r[:3] = final_wp
                        eta_r[3:] = final_orientation
                    else:
                        eta_r, nu_r = self.path_following_controller.generate_reference_trajectories(self.eta_d, self.nu_d, rospy.get_time(), heading_mode, point=point)
                
                # Current estimation mode
                elif self.mode == 'estimate_current':
                    if not self.current_estimator.online:
                        eta_est = self.eta
                        eta_est[3] = 0
                        eta_est[4] = 0
                        self.current_estimator.initailize(eta_est, rospy.get_time())
                    int_x = self.dp_control_system.integral_terms[0]
                    int_y = self.dp_control_system.integral_terms[1]
                    N = rospy.get_param("/guidance_and_control_parameters/current_estimator_number_of_samples")
                    self.current_estimator.estimate_current_velocity(self.eta, N, int_x, int_y, rospy.get_time())
                    eta_r = self.current_estimator.eta_r
                    nu_r = [0, 0, 0, 0, 0, 0]
                    if not self.current_estimator.online:
                        self.dot_eta_c[0] = self.current_estimator.dot_x_c
                        self.dot_eta_c[1] = self.current_estimator.dot_y_c
                        self.mode = 'pose_hold'   

                # Pose hold mode
                elif self.mode == 'pose_hold':
                    eta_r = [0, 0, 0, 0, 0, 0] # Insert desired pose here (roll and pitch do not work atm) [x, y, z, roll, pitch, yaw] NED
                    nu_r = [0, 0, 0, 0, 0, 0]

                # Reference model
                if not self.reference_model.online:
                    self.reference_model.set_initial_conditions(self.eta, self.nu, rospy.get_time())
                eta_d, nu_d, dot_nu_d = self.reference_model.generate_trajectory_for_dp(rospy.get_time(), 1, 1/float(self.publish_rate), eta_r, nu_ref=nu_r)
                self.eta_d = eta_d[0]
                self.nu_d = nu_d[0]

                # Control System
                tau_c = self.dp_control_system.pid_regulate(self.eta, self.nu, eta_d[0], nu_d[0], dot_nu_d[0], rospy.get_time(), dot_eta_c=self.dot_eta_c)

                # Publish virtual target frame
                p = eta_r[:3]
                eul = eta_r[3:]
                q = quaternion_from_euler(eul[0], eul[1], eul[2])
                self.br.sendTransform((p[0], p[1], p[2]), 
                                (q[0], q[1], q[2], q[3]),
                                rospy.Time.now(),
                                "/virtual_target",
                                "/world")
                
                # Publish reference model frame
                p = eta_d[0][:3]
                eul = eta_d[0][3:]
                q = quaternion_from_euler(eul[0], eul[1], eul[2])
                self.br_eta_r.sendTransform((p[0], p[1], p[2]), 
                                (q[0], q[1], q[2], q[3]),
                                rospy.Time.now(),
                                "/reference_model",
                                "/world")
                
                # Publish control forces
                msg = create_wrenchstamped_msg(tau_c, rospy.get_rostime())
                self.rate.sleep()
                self.pub.publish(msg)
            except rospy.ROSInterruptException:
                pass

def extract_from_pose(pose):
    quaternions = pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    position = (pose.position.x, pose.position.y, pose.position.z)
    return [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]

def extract_from_twist(twist):
    linear = (twist.linear.x, twist.linear.y, twist.linear.z)
    angular = (twist.angular.x, twist.angular.y, twist.angular.z)
    return [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]

def publish_path_once(path):
    path_pub = rospy.Publisher('/beluga/guidance_and_control_system/path', Path, queue_size=1)
    msg = Path()
    msg.header.frame_id = '/world'
    for i in range(len(path.path)):
        for j in list(np.linspace(0, 1, 50)):
            p = path.path[i](j)
            psi = path.chi_p[i](j)
            q = quaternion_from_euler(0, 0, psi)
            pose = PoseStamped()
            pose.header.frame_id = '/world'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            msg.poses.append(pose)
    rate = rospy.Rate(10)
    ctrl_c = False
    while not ctrl_c:
        connections = path_pub.get_num_connections()
        if connections > 0:
            path_pub.publish(msg)
            ctrl_c = True
        else:
            rate.sleep()

def create_wrenchstamped_msg(tau, t):
    msg = WrenchStamped()
    msg.header.stamp = t
    msg.header.frame_id = "beluga/base_link_ned"
    msg.wrench.force.x = tau[0]
    msg.wrench.force.y = tau[1]
    msg.wrench.force.z = tau[2]
    msg.wrench.torque.x = tau[3]
    msg.wrench.torque.y = tau[4]
    msg.wrench.torque.z = tau[5]
    return msg

if __name__ == '__main__':
    try:
        node = GuidanceAndControlNode()
        node.publish_control_forces()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
