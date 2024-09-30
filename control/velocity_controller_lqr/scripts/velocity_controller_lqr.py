#!/usr/bin/env python3
import rclpy

import math as math
import control as ct
import numpy as np

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


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

 #----------------------------------------------------------------Controller Node----------------------------------------------------------------
 
class VelocityLQRNode(Node):
    
        def __init__(self):
            super().__init__("input_subscriber")
            self.nucleus_subscriber = self.create_subscription(
                Odometry, "/nucleus/odom", self.nucleus_callback, 10)

            self.guidance_subscriber = self.create_subscription(
                Float32MultiArray, "/guidance/los", self.guidance_callback, 10
            )
            
            self.publisherLQR = self.create_publisher(Wrench, "/thrust/wrench_input", 10)
            self.publisher_states = self.create_publisher(Float32MultiArray, "/velocity/states", 10)
            
            self.control_timer = self.create_timer(0.1, self.LQR_controller)
            self.state_timer = self.create_timer(0.3, self.publish_states)
            self.topic_subscriber = self.create_subscription(Odometry, "/nucleus/odom", self.nucleus_callback, 10)
            
            # TODO: state space model, Anders showed me the chapter in the book from page 55 on for this
            self.M = np.eye(6)*30   # mass matrix with mass = 30kg
            self.A = np.array([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
            self.B = np.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])    # depending on number of control inputs
            self.Q = np.eye(6)  # state cost matrix
            self.R = np.eye(2)  # control cost matrix
            
            # State vector 1. x-position, 2. y-position, 3. z-position, 4. roll, 5. pitch, 6. yaw
            self.states = np.array([0.0], [0.0], [0.0], [0.0], [0.0], [0.0])

#---------------------------------------------------------------Callback Functions---------------------------------------------------------------

def nucleus_callback(self, msg: Odometry):  # callback function
        self.states[0] = msg.pose.pose.position.x
        self.states[1] = msg.pose.pose.position.y
        self.states[2] = msg.pose.pose.position.z
    
        self.states[3], self.states[4], self.states[5] = quaternion_to_euler_angle(msg.pose.pose.orientation.w,
                                                                                   msg.pose.pose.orientation.x,
                                                                                   msg.pose.pose.orientation.y,
                                                                                   msg.pose.pose.orientation.z)
        
def guidance_callback(self, msg: Float32MultiArray):  # Function to read data from guidance
        self.guidance_values = msg.data

#---------------------------------------------------------------Publisher Functions---------------------------------------------------------------

def LQR_controller(self):  # The LQR controller function

        msg = Wrench()
        
        # CT LQR controller from control library python
        self.K = ct.LQR(self.A, self.B, self.Q, self.R)
        # Control input like: u = -self.K * states
        u = np.dot(self.K, self.states)
        
        msg.force.x = u[0]
        msg.force.z = u[1]
        
        msg.torque.y = u[4]
        msg.torque.z = u[5]
        
        self.publisher_controller.publish(msg)

def publish_states(self):
    msg = Float32MultiArray()
    
    # DATA: 1 - Surge, 2 - Heave, 3 - Pitch, 4 - Yaw, 5 - Desired Surge, 6 - Desired Heave, 7 - Desired Pitch, 8 - Desired Yaw
    msg.data = [self.twist[0], self.states[2], self.states[4], self.states[5], self.guidance_values[0], self.guidance_values[1], self.guidance_values[2], self.guidance_values[3]]
    self.publisher_states.publish(msg)

#---------------------------------------------------------------Main Function---------------------------------------------------------------

def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityLQRNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()