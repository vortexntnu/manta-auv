#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from velocity_controller_lqr.velocity_controller_lqr_lib import LQR_controller
from vortex_msgs.msg import LOSGuidance


class VelocityLQRNode(Node):
    def __init__(self):
        super().__init__("velocity_controller_lqr_node")
        
        self.declare_parameter("odom_path", "/nucleus/odom")
        self.declare_parameter("guidance_path", "/guidance/los")
        self.declare_parameter("thrust_path", "/thrust/wrench_input")
        
        odom_path = self.get_parameter("odom_path").value
        guidance_path = self.get_parameter("guidance_path").value
        thrust_path = self.get_parameter("thrust_path").value
        
        # SUBSRCIBERS -----------------------------------
        self.nucleus_subscriber = self.create_subscription(
            Odometry, odom_path, self.nucleus_callback, 20
        )
        self.guidance_subscriber = self.create_subscription(
            LOSGuidance, guidance_path, self.guidance_callback, 20
        )
        
        # PUBLISHERS ------------------------------------
        self.publisherLQR = self.create_publisher(
            Wrench, thrust_path, 10
            )

        # TIMERS ----------------------------------------
        self.control_timer = self.create_timer(
            0.1, self.controller
        )
        self.state_timer = self.create_timer(
            0.1, self.publish_states
            )
        
        # ROS2 SHENNANIGANS with parameters
        self.declare_parameter("max_force", 99.5)
        max_force = self.get_parameter("max_force").value
        
        self.declare_parameter("Q_surge", 75)
        self.declare_parameter("Q_pitch", 175)
        self.declare_parameter("Q_yaw", 175)
        
        self.declare_parameter("R_surge", 0.3)
        self.declare_parameter("R_pitch", 0.4)
        self.declare_parameter("R_yaw", 0.4)
        
        self.declare_parameter("I_surge", 0.3)
        self.declare_parameter("I_pitch", 0.4)
        self.declare_parameter("I_yaw", 0.3)
        
        Q_surge = self.get_parameter("Q_surge").value
        Q_pitch = self.get_parameter("Q_pitch").value
        Q_yaw = self.get_parameter("Q_yaw").value
        
        R_surge = self.get_parameter("R_surge").value
        R_pitch = self.get_parameter("R_pitch").value
        R_yaw = self.get_parameter("R_yaw").value
        
        I_surge = self.get_parameter("I_surge").value
        I_pitch = self.get_parameter("I_pitch").value
        I_yaw = self.get_parameter("I_yaw").value
        
        self.controller = LQR_controller(Q_surge, Q_pitch, Q_yaw, R_surge, R_pitch, R_yaw, I_surge, I_pitch, I_yaw, max_force)
        
        # Only keeping variables that need to be updated inside of a callback
        self.C = np.zeros((3, 3))  # Initialize Coriolis matrix
        self.states = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # State vector 1. surge, 2. pitch, 3. yaw, 4. integral_surge, 5. integral_pitch, 6. integral_yaw
        self.guidance_values = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Array to store guidance values

    # ---------------------------------------------------------------Callback Functions---------------------------------------------------------------

    def nucleus_callback(self, msg: Odometry):  # callback function to read data from nucleus
        dummy, self.states[1], self.states[2] = LQR_controller.quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        self.states[0] = msg.twist.twist.linear.x

        self.C = self.controller.calculate_coriolis_matrix(
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )  # coriolis matrix

    def guidance_callback(
        self, msg: LOSGuidance
    ):  # Function to read data from guidance
        self.guidance_values[0] = msg.surge
        self.guidance_values[1] = msg.pitch
        self.guidance_values[2] = msg.yaw

    # ---------------------------------------------------------------Publisher Functions---------------------------------------------------------------

    def controller(self):  # The LQR controller function
        msg = Wrench()
        
        u = self.controller.calculate_lqr_u(self.C, self.states, self.guidance_values)
        msg.force.x = u[0]
        msg.torque.y = u[1]
        msg.torque.z = u[2]
        
        self.get_logger().info(f"Input values: {msg.force.x}, {msg.torque.y}, {msg.torque.z}")
        
        # Publish the control input
        self.publisherLQR.publish(msg)

# ---------------------------------------------------------------Main Function---------------------------------------------------------------

def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityLQRNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
# Anders er goated