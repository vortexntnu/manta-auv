#!/usr/bin/env python3
import math as math

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

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


#  Function to calculate the coriolis matrix
def calculate_coriolis_matrix(self, pitch_rate, yaw_rate):
    return np.array([[1.629 * pitch_rate, 0], [0, 1.769 * yaw_rate]])


#----------------------------------------------------------------Controller Node----------------------------------------------------------------


class VelocityLQRNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.nucleus_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.nucleus_callback, 10)
        self.guidance_subscriber = self.create_subscription(
            Float32MultiArray, "/guidance/los", self.guidance_callback, 10)
        self.publisherLQR = self.create_publisher(Wrench,
                                                  "/thrust/wrench_input", 10)
        self.publisher_states = self.create_publisher(Float32MultiArray,
                                                      "/velocity/states", 10)

        self.control_timer = self.create_timer(0.1, self.LQR_controller)
        self.state_timer = self.create_timer(0.3, self.publish_states)
        self.topic_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.nucleus_callback, 10)

        self.guidance_values = np.array([np.pi / 4, np.pi / 4
                                         ])  # guidance values TEMPORARY
        self.guidance_values_aug = np.array(
            [np.pi / 4, np.pi / 4, 0.0])  # augmented guidance values TEMPORARY

        # TODO: state space model, Anders showed me the chapter in the book from page 55 on for this
        self.M = np.array([[1.629, 0],
                           [0, 1.769]])  # mass matrix with mass = 30kg
        self.M_inv = np.linalg.inv(self.M)  # inverse of mass matrix
        self.C = np.array([[0, 0], [0, 0]])

        self.A = np.eye(2)  # depending on number of states
        self.B = np.eye(2)  # depending on number of control inputs

        self.A_aug = np.eye(3)  # Augmented A matrix
        self.B_aug = np.eye(3)  # Augmented B matrix

        # LQR controller parameters
        self.Q = np.diag([300, 25])  # state cost matrix
        self.R = np.eye(2) * 0.5  # control cost matrix

        P_I = 0.5  # Augmented state cost for pitch
        Y_I = 0.0  # Augmented state cost for yaw

        self.Q_aug = np.block([[self.Q, np.zeros((2, 1))],
                               [np.zeros((1, 2)),
                                P_I]])  # Augmented state cost matrix

        # State vector 1. pitch, 2. yaw
        self.states = np.array([0, 0])

#---------------------------------------------------------------Callback Functions---------------------------------------------------------------

    def nucleus_callback(self, msg: Odometry):  # callback function

        dummy, self.states[0], self.states[1] = quaternion_to_euler_angle(
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)

    def guidance_callback(
            self,
            msg: Float32MultiArray):  # Function to read data from guidance
        # self.guidance_values = msg.data
        pass

#---------------------------------------------------------------Publisher Functions---------------------------------------------------------------

    def LQR_controller(self):  # The LQR controller function
        msg = Wrench()

        # Coriolis matrix
        #self.C = calculate_coriolis_matrix(self, msg.twist.twist.angular.x, msg.twist.twist.angular.z)

        self.C = calculate_coriolis_matrix(
            1.0, 1.0)  # TEMPORARY linearization of the coriolis matrix
        self.A = -self.M_inv @ self.C
        self.B = self.M_inv

        self.A_aug = np.block([[self.A, np.zeros((2, 1))],
                               [-1, 0, 0]])  # Augmented A matrix for pitch
        self.B_aug = np.block([[self.B], [0, 0]])  # Augmented B matrix

        # CT LQR controller from control library python
        self.K, self.S, self.E = ct.lqr(self.A, self.B, self.Q, self.R)
        self.K_aug, self.S_aug, self.E_aug = ct.lqr(self.A_aug, self.B_aug,
                                                    self.Q_aug, self.R)

        # Control input like: u = -self.K * (desired-states)
        #u = self.K @ (self.guidance_values - self.states)
        self.z = self.z + self.guidance_values[0] - self.states[0]
        u_aug = self.K_aug @ (self.guidance_values_aug -
                              np.block([self.states, self.z]))

        # Control input
        #msg.torque.y = u[0]
        #msg.torque.z = u[1]

        # Augmented control input
        msg.torque.y = u_aug[0]
        msg.torque.z = u_aug[1]

        #Publish the control input
        self.publisherLQR.publish(msg)
        self.get_logger().info(
            f"Pitch input: {msg.torque.y}, Yaw input: {msg.torque.z}")

    def publish_states(self):
        msg = Float32MultiArray()

        # DATA: 1: pitch, 2: yaw, 3: guidance pitch, 4: guidance yaw
        msg.data = [
            self.states[0], self.states[1], self.guidance_values[0],
            self.guidance_values[1]
        ]
        self.publisher_states.publish(msg)


#---------------------------------------------------------------Main Function---------------------------------------------------------------


def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityLQRNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
