#!/usr/bin/python3
# ROS2 Libraries
# Custom Libraries
import internal_status_auv.temperature_sensor_lib
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperaturePublisher(Node):
    def __init__(self) -> None:
        # Pressure sensor setup ----------
        self.temperature = internal_status_auv.temperature_sensor_lib.TemperatureSensor()

        # Node setup ----------
        super().__init__("temperature_sensor_publisher")

        # Create publishers ----------
        self.publisher_temperature = self.create_publisher(Float32, "/auv/temperature", 5)

        # Data gathering cycle ----------
        self.temperature = 0.0

        self.declare_parameter("internal_status.temperature_read_rate", 0.1)  # Providing a default value 0.1 => 10 second delay per data gathering
        read_rate = self.get_parameter("internal_status.temperature_read_rate").get_parameter_value().double_value
        timer_period = 1.0 / read_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Watchdog for anomalies ----------
        self.logger = get_logger("temperature_sensor")

        self.declare_parameter("internal_status.temperature_critical_level", 90.0)
        self.temperature_critical_level = self.get_parameter("internal_status.temperature_critical_level").get_parameter_value().double_value

        self.declare_parameter("internal_status.temperature_warning_rate", 0.1)
        warning_rate = self.get_parameter("internal_status.temperature_warning_rate").get_parameter_value().double_value
        warning_timer_period = 1.0 / warning_rate
        self.warning_timer = self.create_timer(warning_timer_period, self.warning_timer_callback)

        # Debugging ----------
        self.get_logger().info('"temperature_sensor_publisher" has been started')

    def timer_callback(self) -> None:
        """
        Callback function triggered by the main timer.

        This function retrieves the temperature data from the sensor
        and publishes it to the "/auv/temperature" topic.
        """
        # Get temperature data
        self.temperature = self.temperature.get_temperature()

        # Publish temperature data
        temperature_msg = Float32()
        temperature_msg.data = self.temperature
        self.publisher_temperature.publish(temperature_msg)

    def warning_timer_callback(self) -> None:
        """
        Callback function triggered by the warning timer.

        This function checks if the temperature exceeds the critical level.
        If so, a fatal warning is logged indicating a possible overheating situation.
        """
        if self.temperature > self.temperature_critical_level:
            self.logger.fatal(f"WARNING: Temperature inside the Drone to HIGH: {self.temperature} *C! Drone might be overheating!")


def main(args: list = None) -> None:
    """
    Main function to initialize and spin the ROS2 node.

    This function initializes the rclpy library, creates an instance of the
    TemperaturePublisher node, and starts spinning to keep the node running
    and publishing temperature data.

    Args:
    -----
    args : list, optional
        Arguments passed to the node. Default is None.
    """
    rclpy.init(args=args)

    temperature_publisher = TemperaturePublisher()

    rclpy.spin(temperature_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temperature_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
