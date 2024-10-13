#!/usr/bin/env python3
import math as math

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from vortex_msgs.msg import LOSGuidance

import control as ct


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


# A function to convert the angle to the smallest signed angle
def ssa(angle):
    if angle > np.pi:
        angle -= 2 * np.pi
    elif angle < -np.pi:
        angle += 2 * np.pi
    return angle


#  Function to calculate the coriolis matrix
def calculate_coriolis_matrix(pitch_rate, yaw_rate, sway, heave):
    return np.array(
        [[0.2, -30 * sway * 0.01, -30 * heave * 0.01], [30 * sway * 0.01, 0, 1.629 * pitch_rate], [30 * heave * 0.01, 1.769 * yaw_rate, 0]]
    )


invalid_force = 95.0
source_from_abu = True

# ----------------------------------------------------------------Controller Node----------------------------------------------------------------


class VelocityLQRNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.nucleus_subscriber = self.create_subscription(Odometry, "/nucleus/odom", self.nucleus_callback, 10)
        self.guidance_subscriber = self.create_subscription(LOSGuidance, "/guidance/los", self.guidance_callback, 10)
        self.publisherLQR = self.create_publisher(Wrench, "/thrust/wrench_input", 10)
        self.publisher_states = self.create_publisher(Float32MultiArray, "/velocity/states", 10)

        self.control_timer = self.create_timer(0.1, self.LQR_controller)
        self.state_timer = self.create_timer(0.3, self.publish_states)
        self.sinusoid_timer = self.create_timer(0.1, self.timer_callback)
        self.topic_subscriber = self.create_subscription(Odometry, "/nucleus/odom", self.nucleus_callback, 10)

        self.guidance_values = np.array([0.3, -np.pi / 8, -np.pi / 4])  # guidance values TEMPORARY
        self.guidance_values_aug = np.array([0.3, -np.pi / 8, -np.pi / 4, 0.0, 0.0, 0.0])  # augmented guidance values TEMPORARY

        # TODO: state space model, Anders showed me the chapter in the book from page 55 on for this
        self.M = np.array([[30, 0.6, 0], [0.6, 1.629, 0], [0, 0, 1.769]])  # mass matrix with mass = 30kg

        self.M_inv = np.linalg.inv(self.M)  # inverse of mass matrix

        self.C = np.zeros((3, 3))  # Coriolis matrix
        self.A = np.eye(3)  # depending on number of states
        self.B = np.eye(3)  # depending on number of control inputs

        self.A_aug = np.eye(6)  # Augmented A matrix
        self.B_aug = np.eye(6, 3)  # Augmented B matrix

        # LQR controller parameters
        self.Q = np.diag([75, 175, 175])  # state cost matrix
        self.R = np.diag([0.1, 0.3, 0.1])  # control cost matrix

        self.I_surge = 0.40  # Augmented state cost for surge
        self.I_pitch = 0.35  # Augmented state cost for pitch
        self.I_yaw = 0.35  # Augmented state cost for yaw

        self.Q_aug = np.block(
            [[self.Q, np.zeros((3, 3))], [np.zeros((3, 3)), np.diag([self.I_surge, self.I_pitch, self.I_yaw])]]
        )  # Augmented state cost matrix

        self.z_surge = 0.0
        self.z_pitch = 0.0
        self.z_yaw = 0.0  # Augmented states for surge, pitch and yaw

        self.surge_windup = False  # Windup variable
        self.pitch_windup = False  # Windup variable
        self.yaw_windup = False  # Windup variable

        # State vector 1. surge, 2. pitch, 3. yaw
        self.states = np.array([0.0, 0.0, 0.0])
        self.t = 0.0

    # ---------------------------------------------------------------Callback Functions---------------------------------------------------------------

    def nucleus_callback(self, msg: Odometry):  # callback function

        dummy, self.states[1], self.states[2] = quaternion_to_euler_angle(
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        )

        self.states[0] = msg.twist.twist.linear.x

        self.C = calculate_coriolis_matrix(
            msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        )  # coriolis matrix

    def guidance_callback(self, msg: LOSGuidance):  # Function to read data from guidance
        if source_from_abu == True:
            self.guidance_values[0] = msg.surge
            self.guidance_values[1] = msg.pitch
            self.guidance_values[2] = msg.yaw

            self.guidance_values_aug[0] = msg.surge
            self.guidance_values_aug[1] = msg.pitch
            self.guidance_values_aug[2] = msg.yaw

        self.get_logger().info(f"Guidance values: {self.guidance_values}")

    def timer_callback(self):  # Timer callback function
        if source_from_abu == False:
            self.guidance_values[1] = -abs((np.pi / 4) * np.cos(self.t / 75))
            self.guidance_values_aug[1] = -abs((np.pi / 4) * np.cos(self.t / 75))

            self.guidance_values[2] = (np.pi / 2) * np.cos(self.t / 75)
            self.guidance_values_aug[2] = (np.pi / 2) * np.cos(self.t / 75)
            self.t += 1

    # ---------------------------------------------------------------Publisher Functions---------------------------------------------------------------

    def LQR_controller(self):  # The LQR controller function
        msg = Wrench()

        self.A = -self.M_inv @ self.C
        self.B = self.M_inv

        # Augment the A and B matrices for integrators for surge, pitch, and yaw
        self.A_aug = np.block([[self.A, np.zeros((3, 3))], [-np.eye(3), np.zeros((3, 3))]])  # Integrators added for surge, pitch, and yaw
        self.B_aug = np.block([[self.B], [np.zeros((3, 3))]])  # Control input does not affect integrators directly

        # CT LQR controller from control library python
        self.K, self.S, self.E = ct.lqr(self.A, self.B, self.Q, self.R)
        self.K_aug, self.S_aug, self.E_aug = ct.lqr(self.A_aug, self.B_aug, self.Q_aug, self.R)

        surge_error = self.guidance_values[0] - self.states[0]  # Surge error no need for angle wrapping
        pitch_error = ssa(self.guidance_values[1] - self.states[1])  # Apply ssa to pitch error
        yaw_error = ssa(self.guidance_values[2] - self.states[2])  # Apply ssa to yaw error

        # Update the integrators for surge, pitch, and yaw

        if self.surge_windup == False:
            self.z_surge += self.I_surge * surge_error  # surge integrator
        else:
            self.z_surge += 0.0

        if self.pitch_windup == False:
            self.z_pitch += self.I_pitch * pitch_error * 2  # pitch integrator
        else:
            self.z_pitch += 0.0

        if self.yaw_windup == False:
            self.z_yaw += self.I_yaw * yaw_error  # yaw integrator
        else:
            self.z_yaw += 0.0

        # Augmented state vector including integrators
        u_aug = -self.K_aug @ (-surge_error, -pitch_error, -yaw_error, self.z_surge, self.z_pitch, self.z_yaw)  # Augmented control input

        # --------------------------------------------- SURGE ANTIWINDUP -------------------------------------------------------
        if abs(u_aug[0]) > invalid_force:
            self.surge_windup = True
            if u_aug[0] > 0:
                msg.force.x = invalid_force
            else:
                msg.force.x = -invalid_force
        else:
            self.surge_windup = False
            msg.force.x = u_aug[0]

        # --------------------------------------------- PITCH ANTIWINDUP -------------------------------------------------------
        if abs(u_aug[1]) > invalid_force:
            self.pitch_windup = True
            if u_aug[1] > 0:
                msg.torque.y = invalid_force
            else:
                msg.torque.y = -invalid_force
        else:
            self.pitch_windup = False
            msg.torque.y = u_aug[1]

        # ----------------------------------------------- YAW ANTIWINDUP ---------------------------------------------------------
        if abs(u_aug[2]) > invalid_force:
            self.pitch_windup_windup = True
            if u_aug[2] > 0:
                msg.torque.z = invalid_force
            else:
                msg.torque.z = -invalid_force
        else:
            self.pitch_windup = False
            msg.torque.z = u_aug[2]

        # Publish the control input
        self.publisherLQR.publish(msg)

        # self.get_logger().info(
        #     f"Surge input: {msg.force.x} Pitch input: {msg.torque.y}, Yaw input: {msg.torque.z}"
        # )

    def publish_states(self):
        msg = Float32MultiArray()

        # DATA: 1: surge 2: pitch, 3: yaw, 4: guidance surge 5: guidance pitch, 6: guidance yaw
        msg.data = [
            self.states[0],
            self.states[1],
            self.states[2],
            self.guidance_values[0],
            self.guidance_values[1],
            self.guidance_values[2],
            abs(ssa(self.states[1] - self.guidance_values[1])),
            abs(ssa(self.states[2] - self.guidance_values[2])),
        ]

        self.publisher_states.publish(msg)


# ---------------------------------------------------------------Main Function---------------------------------------------------------------


def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityLQRNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
