#!/usr/bin/env python3
import math as math

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node


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


# TOLERANCES (Have not given this much thought yet)
pitch_tolerance = 0
yaw_tolerance = 0.01
radius_tolerance = 0.3
heave_tolerance = 0.5

# Still experimenting with different values, we might want to try an integral term for the pitch as we have a constant buoyancy force to counteract
Kp_linear = 3
Kp_heave = 5
Kp_yaw = 3
Kp_pitch = 0


class VelocityNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.topic_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.subscribe_callback, 10)
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
        self.goal_position = np.array([20.0, -5.0, -5.0])

    # A function to convert the angle to the smallest signed angle
    def ssa(self, angle):
        if angle > np.pi:
            angle = angle - 2 * np.pi
        elif angle < -np.pi:
            angle = angle + 2 * np.pi
        return angle

    def subscribe_callback(self, msg: Odometry):  # callback function
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        # GETTING THE GOAL POSITION
        self.goal_yaw = np.arctan2(
            self.goal_position[2] - self.position[2],
            self.goal_position[0] - self.position[0],
        )
        self.goal_pitch = np.arctan2(
            self.goal_position[1] - self.position[1],
            self.goal_position[0] - self.position[0],
        )

        self.heave_error = self.goal_position[2] - self.position[2]

        # GETTING THE ERROR OF ALL STATES
        self.yaw_error = (self.goal_yaw - quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )[2])

        self.pitch_error = (self.goal_pitch - quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )[1])

        self.radius_error = np.linalg.norm(self.position - self.goal_position)

    def publish_callback(self):  # The controller function
        msg = Wrench()

        # MAIN CONTROL LOOP

        msg.torque.z = -Kp_yaw * self.yaw_error

        if abs(self.yaw_error) < yaw_tolerance:
            msg.force.z = -Kp_heave * self.heave_error

        if (abs(self.heave_error) < heave_tolerance
                and abs(self.yaw_error) < yaw_tolerance):
            if self.radius_error < radius_tolerance:
                msg.force.x = 0.0
            else:
                msg.force.x = Kp_linear * self.radius_error * np.cos(
                    self.yaw_error)

        self.get_logger().info(
            f"\nyaw_error: {self.ssa(self.yaw_error)}, \npitch_error: {self.ssa(self.pitch_error)}, \nradius_error: {self.radius_error}"
        )
        self.publisher.publish(msg)


def main(args=None):  # main function
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
