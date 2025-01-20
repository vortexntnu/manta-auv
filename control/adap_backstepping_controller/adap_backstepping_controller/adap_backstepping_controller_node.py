#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from vortex_msgs.msg import ReferenceFilter

from adap_backstepping_controller.adap_backstepping_controller import AdaptiveBackstep

qos_profile = QoSProfile(
    depth=1,
    history=qos_profile_sensor_data.history,
    reliability=qos_profile_sensor_data.reliability,
)


class HybridPathControllerNode(Node):
    def __init__(self):
        super().__init__("adap_backstepping_controller_node")

        # # Se over her
        self.state_subscriber_ = self.state_subscriber_ = self.create_subscription(
            Odometry, '/orca/odom', self.state_callback, qos_profile=qos_profile
        )
        self.hpref_subscriber_ = self.create_subscription(
            ReferenceFilter,
            '/dp/reference',
            self.reference_callback,
            qos_profile=qos_profile,
        )
        self.wrench_publisher_ = self.create_publisher(
            Wrench, '/thrust/wrench_input', 10
        )

        # Transform parameters to diagonal matrices
        K1 = np.diag([20.5, 15.5, 20.5, 1.2, 6.0, 2.5])
        K2 = np.diag([30.5, 25.5, 30.5, 2.6, 10.0, 6.5])

        # Reshape M to a 3x3 matrix
        M_RB = np.array(
            [
                [30, 0, 0, 0, 0, 0.6],
                [0, 30, 0, 0, -0.6, 0.3],
                [0, 0, 30, 0.6, 0.3, 0],
                [0, 0, 0.6, 0.68, 0, 0],
                [0, -0.6, 0.3, 0, 3.32, 0],
                [0.6, 0.3, 0, 0, 0, 3.34],
            ]
        )
        #                    [0, 0, 0.6, 0.64278, 0, 0],
        #                    [0, -0.6, 0.3, 0, 0.64290, 0],
        #                    [0.6, 0.3, 0, 0, 0, 1.1688]
        # Create controller object
        self.AB_controller_ = AdaptiveBackstep(K1, K2, M_RB)

        controller_period = 0.04
        self.controller_timer_ = self.create_timer(
            controller_period, self.controller_callback
        )

        self.get_logger().info("hybridpath_controller_node started")

    def state_callback(self, msg: Odometry):
        """Callback function for the Odometry message. This function saves the state message."""
        self.state_odom = msg

    def reference_callback(self, msg: ReferenceFilter):
        self.get_logger().info(f"Received reference: {msg}")
        self.reference = msg

    def controller_callback(self):
        """Callback function for the controller timer. This function calculates the control input and publishes the control input."""
        if hasattr(self, 'killswitch_on_') and self.killswitch_on_:
            return

        if hasattr(self, 'software_mode_') and self.software_mode_ != "autonomous mode":
            return

        if hasattr(self, 'state_odom') and hasattr(self, 'reference'):
            control_input = self.AB_controller_.control_law(
                self.state_odom, self.reference
            )
            wrench_msg = Wrench()
            wrench_msg.force.x = control_input[0]
            wrench_msg.force.y = control_input[1]
            wrench_msg.force.z = control_input[2]
            wrench_msg.torque.x = control_input[3]
            wrench_msg.torque.y = control_input[4]
            wrench_msg.torque.z = control_input[5]
            self.wrench_publisher_.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridPathControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
