#!/usr/bin/env python3
# ROS2 Libraries
# Custom Libraries
import internal_status_auv.power_sense_module_lib
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Float32


class PowerSenseModulePublisher(Node):
    def __init__(self) -> None:
        # Node setup ----------
        super().__init__("power_sense_module_publisher")
        self.psm = internal_status_auv.power_sense_module_lib.PowerSenseModule()

        # Create publishers ----------
        self.publisher_current = self.create_publisher(Float32, "/auv/power_sense_module/current", 5)
        self.publisher_voltage = self.create_publisher(Float32, "/auv/power_sense_module/voltage", 5)

        # Data gathering cycle ----------
        self.current = 0.0
        self.voltage = 0.0

        self.declare_parameter(
            "internal_status.power_sense_module_read_rate", 10.0
        )  # Providing a default value 10.0 => 0.1 second delay per data gathering
        read_rate = self.get_parameter("internal_status.power_sense_module_read_rate").get_parameter_value().double_value
        read_timer_period = 1.0 / read_rate
        self.read_timer = self.create_timer(read_timer_period, self.read_timer_callback)

        # Watchdog for anomalies ----------
        self.logger = get_logger("power_sense_module")

        self.declare_parameter("internal_status.voltage_min", 14.5)
        self.voltage_min = self.get_parameter("internal_status.voltage_min").get_parameter_value().double_value

        self.declare_parameter("internal_status.voltage_max", 16.8)
        self.voltage_max = self.get_parameter("internal_status.voltage_max").get_parameter_value().double_value

        self.declare_parameter("internal_status.power_sense_module_warning_rate", 0.1)
        warning_rate = self.get_parameter("internal_status.power_sense_module_warning_rate").get_parameter_value().double_value
        warning_timer_period = 1.0 / warning_rate
        self.warning_timer = self.create_timer(warning_timer_period, self.warning_timer_callback)

        # Debugging ----------
        self.get_logger().info('"power_sense_module_publisher" has been started')

    def read_timer_callback(self) -> None:
        """
        Callback function triggered by the read timer.

        This function retrieves the current and voltage data from the power sense module
        and publishes it to the corresponding ROS2 topics.
        """
        # Get the PSM data
        self.current = self.psm.get_current()
        self.voltage = self.psm.get_voltage()

        # Publish PSM data
        current_msg = Float32()
        voltage_msg = Float32()

        current_msg.data = self.current
        voltage_msg.data = self.voltage

        self.publisher_current.publish(current_msg)  # publish current value to the "current topic"
        self.publisher_voltage.publish(voltage_msg)  # publish voltage value to the "voltge topic"

    def warning_timer_callback(self) -> None:
        """
        Callback function triggered by the warning timer.

        This function checks if the voltage levels are outside of the acceptable range
        and logs a warning if the voltage is either too low or too high.
        """
        if self.voltage < self.voltage_min:
            self.logger.fatal(f"WARNING: Battery Voltage to LOW at {self.voltage} V")
        elif self.voltage > self.voltage_max:
            self.logger.fatal(f"WARNING: Battery Voltage to HIGH at {self.voltage} V")


def main(args: list = None) -> None:
    """
    Main function to initialize and spin the ROS2 node.

    This function initializes the rclpy library, creates an instance of the
    PowerSenseModulePublisher node, and starts spinning to keep the node running
    and publishing current and voltage data.

    Args:
    -----
    args : list, optional
        Arguments passed to the node. Default is None.
    """
    rclpy.init(args=args)

    power_sense_module_publisher = PowerSenseModulePublisher()

    rclpy.spin(power_sense_module_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    power_sense_module_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
