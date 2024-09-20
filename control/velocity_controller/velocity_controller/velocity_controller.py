#!usr/bin/env/python 3
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VelocityNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.topic_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.subscribe_callback, 10)

    def subscribe_callback(self, msg: Odometry):
        self.get_logger().info(
            f"position_x: {msg.pose.pose.position.x}\n_y: {msg.pose.pose.position.y}\n_z: {msg.pose.pose.position.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
