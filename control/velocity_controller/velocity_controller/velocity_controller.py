#!usr/bin/env/python 3
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VelocityNode(Node):

    def __init__(self):
        super().__init__("input_subscriber")
        self.topic_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.subscribe_callback, 10)
        self.publisher = self.create_publisher(Wrench, "/thrust/wrench_input",
                                               10)
        self.timer = self.create_timer(0.1, self.publish_callback)

    def subscribe_callback(self, msg: Odometry):  #callback function
        self.get_logger().info(
            f"\n_x: {msg.pose.pose.position.x}\n_y: {msg.pose.pose.position.y}\n_z: {msg.pose.pose.position.z}"
        )

    def publish_callback(self):  #callback function
        msg = Wrench()
        msg.force.x = 1.0
        msg.torque.y = 1.0
        msg.torque.z = 1.0
        self.publisher.publish(msg)


def main(args=None):  #main function
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
