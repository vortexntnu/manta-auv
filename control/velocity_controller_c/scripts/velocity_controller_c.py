#!/usr/bin/env python3
import math as math

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


#A function to convert orientation quaternions to euler angles
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

def rotate_point(x, y, z , theta, phi, psi):
    # Rotation matrix
    R = np.array([[np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi)],
                  [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi)],
                  [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])
    # Point to be rotated
    P = np.array([x, y, z])
    # Rotated point
    P_rot = np.dot(R, P)
    
    return round(P_rot[0],12) , round(P_rot[1],12), round(P_rot[2],12)

class VelocityNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.nucleus_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.nucleus_callback, 10)

        self.guidance_subscriber = self.create_subscription(
            Float32MultiArray, "/guidance/los", self.guidance_callback, 10
        )
        
        self.publisher_controller = self.create_publisher(Wrench, "/thrust/wrench_input", 10)
        self.publisher_states = self.create_publisher(Float32MultiArray, "/Velocity/States", 10)
        self.control_timer = self.create_timer(0.1, self.publish_callback)
        self.state_timer = self.create_timer(0.3, self.publish_states)
        self.topic_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.nucleus_callback, 10)
        self.publisher = self.create_publisher(Wrench, "/thrust/wrench_input",
                                               10)
        self.timer = self.create_timer(0.1, self.publish_callback)

        # Defining all the errors of all the states
        self.pitch_error = 0.0
        self.heave_error = 0.0
        self.yaw_error = 0.0
        self.radius_error = 0.0
        self.heave_error = 0.0
        self.goal_pitch = 0.0
        self.goal_yaw = 0.0

        # Arbitrary goal position as its Abu's task to feed us this information
        self.position = np.array([0.0, 0.0, 0.0])
        self.twist = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.rotated_point_position = np.array([0.0, 0.0, 0.0])
        self.abu_values = np.array([0.3, -5.0 , np.pi/4, np.pi/4])
        
        self.Kp_surge = 20.0
        self.Kp_heave = 10.0
        self.Kp_pitch = 2.5
        self.Kp_yaw = 2.5
        
        self.Ti_pitch = 0.05
        self.Ti_yaw = 0.001
        self.Ti_surge = 1.0
        self.Ti_heave = 0.5
        self.pitch_integral_sum = 0.0
        self.yaw_integral_sum = 0.0
        self.surge_integral_sum = 0.0
        self.heave_integral_sum = 0.0
        
        self.heave_error = 0.0
        self.pitch_error = 0.0
        self.yaw_error = 0.0
        self.surge_error = 0.0
        
        self.get_logger().info("Velocity Controller Node has been started")

    def nucleus_callback(self, msg: Odometry):  # callback function
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        self.twist = np.array([
            msg.twist.twist.linear.x, msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        X, Y, Z = quaternion_to_euler_angle(msg.pose.pose.orientation.w,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z)
        self.orientation = np.array([X, Y, Z])
        
        # self.rotated_point_position[0], self.rotated_point_position[1], self.rotated_point_position[2] = rotate_point(self.position[0], self.position[1], self.position[2], Y, X, Z)
        
    def guidance_callback(self, msg: Float32MultiArray):  # callback function
        self.abu_values = msg.data

        
    def publish_callback(self):  # The controller function

        self.surge_error = self.abu_values[0] - self.twist[0]
        self.heave_error = self.abu_values[1] - self.rotated_point_position[2]
        self.pitch_error = ssa(self.abu_values[2] - self.orientation[1])
        self.yaw_error = ssa(self.abu_values[3] - self.orientation[2])

        msg = Wrench()
        
        # if abs(self.pitch_error) < 0.05 and abs(self.yaw_error < 0.05):
            
        #     if abs(self.surge_error) > 0.005:
        #         self.surge_integral_sum += self.surge_error
        #     else:
        #         self.surge_integral_sum += 0.0
        #     msg.force.x = self.surge_error*self.Kp_surge + self.surge_integral_sum * self.Ti_surge         # SURGE
        
        # else:
        #     self.surge_integral_sum = 0.0
        #     msg.force.x = 0.0

        self.pitch_integral_sum += self.pitch_error
        msg.torque.y = -(self.pitch_error * self.Kp_pitch + self.pitch_integral_sum * self.Ti_pitch)   # PITCH 
            
        if abs(self.yaw_error) > 0.005:
            self.yaw_integral_sum += self.yaw_error
        else:
            self.yaw_integral_sum = 0.0
            
        msg.torque.z = -(self.yaw_error * self.Kp_yaw + self.yaw_integral_sum * self.Ti_yaw)            # YAW
        
        # if abs(self.heave_error) > 0.005:
        #     self.heave_integral_sum += self.heave_error
        # else:
        #     self.heave_integral_sum = 0.0
        # msg.force.z = self.heave_error*self.Kp_heave + self.heave_integral_sum * self.Ti_heave
        
        self.publisher_controller.publish(msg)
        
    def publish_states(self):
        msg = Float32MultiArray()
        msg.data = [
            self.abu_values[2], self.abu_values[3], self.orientation[1],
            self.orientation[2], self.abu_values[0], self.abu_values[1],
            self.twist[0], self.twist[2]
        ]
        self.publisher_states.publish(msg)


def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
