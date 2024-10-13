#!/usr/bin/env python3
# ROS2 Libraries
# Custom Libraries
import internal_status_auv.pressure_sensor_lib
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Float32


class PressurePublisher(Node):
    def __init__(self) -> None:
        # Pressure sensor setup ----------
        self.pressure = internal_status_auv.pressure_sensor_lib.PressureSensor()

        # Node setup ----------
        super().__init__("pressure_sensor_publisher")

        # Create publishers ----------
        self.publisher_pressure = self.create_publisher(Float32, "/auv/pressure", 5)

        # Data gathering cycle ----------
        self.pressure = 0.0

        self.declare_parameter("internal_status.pressure_read_rate", 1.0)  # Providing a default value 1.0 => 1 second delay per data gathering
        read_rate = self.get_parameter("internal_status.pressure_read_rate").get_parameter_value().double_value
        timer_period = 1.0 / read_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Watchdog for anomalies ----------
        self.logger = get_logger("pressure_sensor")

        self.declare_parameter("internal_status.pressure_critical_level", 1000.0)
        self.pressure_critical_level = self.get_parameter("internal_status.pressure_critical_level").get_parameter_value().double_value

        self.declare_parameter("internal_status.pressure_warning_rate", 0.1)
        warning_rate = self.get_parameter("internal_status.pressure_warning_rate").get_parameter_value().double_value
        warning_timer_period = 1.0 / warning_rate
        self.warning_timer = self.create_timer(warning_timer_period, self.warning_timer_callback)

        # Debugging ----------
        self.get_logger().info('"pressure_sensor_publisher" has been started')

    def timer_callback(self) -> None:
        """
        Callback function triggered by the main timer.

        This function retrieves the pressure data from the sensor
        and publishes it to the "/auv/pressure" topic.
        """
        # Get pressure data
        self.pressure = self.pressure.get_pressure()

        # Publish pressure data
        pressure_msg = Float32()
        pressure_msg.data = self.pressure
        self.publisher_pressure.publish(pressure_msg)

    def warning_timer_callback(self) -> None:
        """
        Callback function triggered by the warning timer.

        This function checks if the pressure exceeds the critical level.
        If so, a fatal warning is logged indicating a possible leak in the AUV.
        """
        if self.pressure > self.pressure_critical_level:
            self.logger.fatal(f"WARNING: Internal pressure to HIGH: {self.pressure} hPa! Drone might be LEAKING!")


def main(args: list = None) -> None:
    """
    Main function to initialize and spin the ROS2 node.

    This function initializes the rclpy library, creates an instance of
    the PressurePublisher node, and starts spinning to keep the node
    running and publishing pressure data.

    Args:
    -----
    args : list, optional
        Arguments passed to the node. Default is None.
    """
    rclpy.init(args=args)

    pressure_publisher = PressurePublisher()

    rclpy.spin(pressure_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pressure_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
