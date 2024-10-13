#!/usr/bin/env python3
# ROS2 Libraries
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from thruster_interface_auv.thruster_interface_auv_driver_lib import (
    ThrusterInterfaceAUVDriver,
)

# Custom libraries
from vortex_msgs.msg import ThrusterForces


class ThrusterInterfaceAUVNode(Node):
    def __init__(self) -> None:
        # Initialize and name the node process running
        super().__init__("thruster_interface_auv_node")

        # Create a subscriber that takes data from thruster forces
        # Then convert this Forces into PWM signals and control the thrusters
        # Publish PWM values as deebuging feature
        self.thruster_forces_subscriber = self.create_subscription(ThrusterForces, "thrust/thruster_forces", self._thruster_forces_callback, 10)
        self.thruster_pwm_publisher = self.create_publisher(Int16MultiArray, "pwm", 10)

        # Get thruster mapping, direction, offset and clamping parameters
        self.declare_parameter("propulsion.thrusters.thruster_to_pin_mapping", [7, 6, 5, 4, 3, 2, 1, 0])
        self.declare_parameter("propulsion.thrusters.thruster_direction", [1, 1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter("propulsion.thrusters.thruster_PWM_offset", [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter(
            "propulsion.thrusters.thruster_PWM_min",
            [1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100],
        )
        self.declare_parameter(
            "propulsion.thrusters.thruster_PWM_max",
            [1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900],
        )

        self.declare_parameter("propulsion.thrusters.thrust_update_rate", 10.0)

        self.thruster_mapping = self.get_parameter("propulsion.thrusters.thruster_to_pin_mapping").value
        self.thruster_direction = self.get_parameter("propulsion.thrusters.thruster_direction").value
        self.thruster_pwm_offset = self.get_parameter("propulsion.thrusters.thruster_PWM_offset").value
        self.thruster_pwm_min = self.get_parameter("propulsion.thrusters.thruster_PWM_min").value
        self.thruster_pwm_max = self.get_parameter("propulsion.thrusters.thruster_PWM_max").value
        self.thrust_timer_period = 1.0 / self.get_parameter("propulsion.thrusters.thrust_update_rate").value

        # Initialize thruster driver
        self.thruster_driver = ThrusterInterfaceAUVDriver(
            ros2_package_name_for_thruster_datasheet=get_package_share_directory("thruster_interface_auv"),
            thruster_mapping=self.thruster_mapping,
            thruster_direction=self.thruster_direction,
            thruster_pwm_offset=self.thruster_pwm_offset,
            pwm_min=self.thruster_pwm_min,
            pwm_max=self.thruster_pwm_max,
        )

        # Start clock timer for driving thrusters every 0.2 seconds
        # Declare "self.thruster_forces_array" in case no topic comes in at the first possible second
        self.thruster_forces_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.timer = self.create_timer(self.thrust_timer_period, self._timer_callback)

        # Debugging
        self.get_logger().info('"thruster_interface_auv_node" has been started')

    def _thruster_forces_callback(self, msg: ThrusterForces) -> None:
        # Get data of the forces published
        self.thruster_forces_array = msg.thrust

    def _timer_callback(self) -> None:
        # Send thruster forces to be converted into PWM signal and sent to control the thrusters
        # PWM signal gets saved and is published in the "/pwm" topic as a debugging feature to see if everything is alright with the PWM signal
        thruster_pwm_array = self.thruster_driver.drive_thrusters(self.thruster_forces_array)

        pwm_message = Int16MultiArray()
        pwm_message.data = thruster_pwm_array
        self.thruster_pwm_publisher.publish(pwm_message)


def main(args: list = None) -> None:
    """
    Entry point for the thruster interface AUV node.

    This function initializes the ROS 2 client library, creates an instance of the
    ThrusterInterfaceAUVNode, and starts spinning the node to process callbacks.
    Upon shutdown, it destroys the node and shuts down the ROS 2 client library.

    Args:
        args (list, optional): Command line arguments passed to the ROS 2 client library.
                               Defaults to None.
    """
    # Initialize
    rclpy.init(args=args)

    # Running
    thruster_interface_auv_node = ThrusterInterfaceAUVNode()
    rclpy.spin(thruster_interface_auv_node)

    # Shutdown
    thruster_interface_auv_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
