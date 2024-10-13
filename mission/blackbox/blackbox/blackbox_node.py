#!/usr/bin/env python3

# ROS2 Libraries
import array

import rclpy
from ament_index_python.packages import get_package_share_directory
from blackbox.blackbox_log_data import BlackBoxLogData
from rclpy.node import Node

# ROS2 Topic Libraries
from std_msgs.msg import Float32, Int16MultiArray

# Custom Libraries
from vortex_msgs.msg import ThrusterForces


class BlackBoxNode(Node):
    def __init__(self) -> None:
        # Start the ROS2 Node ----------
        super().__init__("blackbox_node")

        # Initialize sunscribers ----------
        self.psm_current_subscriber = self.create_subscription(Float32, "/auv/power_sense_module/current", self.psm_current_callback, 1)
        self.psm_current_data = 0.0

        self.psm_voltage_subscriber = self.create_subscription(Float32, "/auv/power_sense_module/voltage", self.psm_voltage_callback, 1)
        self.psm_voltage_data = 0.0

        self.pressure_subscriber = self.create_subscription(Float32, "/auv/pressure", self.pressure_callback, 1)
        self.pressure_data = 0.0

        self.temperature_subscriber = self.create_subscription(Float32, "/auv/temperature", self.temperature_callback, 1)
        self.temperature_data = 0.0

        self.thruster_forces = self.create_subscription(ThrusterForces, "/thrust/thruster_forces", self.thruster_forces_callback, 1)
        self.thruster_forces_data = array.array("f", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.pwm = self.create_subscription(Int16MultiArray, "/pwm", self.pwm_callback, 1)
        self.pwm_data = array.array("i", [0, 0, 0, 0, 0, 0, 0, 0])

        # Initialize logger ----------
        # Get package directory location
        ros2_package_directory_location = get_package_share_directory("blackbox")
        ros2_package_directory_location = ros2_package_directory_location + "/../../../../"  # go back to workspace
        ros2_package_directory_location = ros2_package_directory_location + "src/vortex-auv/mission/blackbox/"  # Navigate to this package

        # Make blackbox logging file
        self.blackbox_log_data = BlackBoxLogData(ros2_package_directory=ros2_package_directory_location)

        # Logs all the newest data 10 times per second
        self.declare_parameter("blackbox.data_logging_rate", 1.0)  # Providing a default value 1.0 => 1 samplings per second, very slow
        data_logging_rate = self.get_parameter("blackbox.data_logging_rate").get_parameter_value().double_value
        timer_period = 1.0 / data_logging_rate
        self.logger_timer = self.create_timer(timer_period, self.logger)

        # Debugging ----------
        self.get_logger().info(
            "Started logging data for topics: \n"
            "/auv/power_sense_module/current [Float32] \n"
            "/auv/power_sense_module/voltage [Float32] \n"
            "/auv/pressure [Float32] \n"
            "/auv/temperature [Float32] \n"
            "/thrust/thruster_forces [ThrusterForces] \n"
            "/pwm [Float32] \n"
        )

    # Callback Methods ----------
    def psm_current_callback(self, msg: Float32) -> None:
        """
        Callback function for the power sense module (PSM) current topic.

        This function is called whenever a new message is received on the
        "/auv/power_sense_module/current" topic. It updates the internal
        state with the latest current data.

        Args:
            msg (std_msgs.msg.Float32): The message containing the current data.
        """
        self.psm_current_data = msg.data

    def psm_voltage_callback(self, msg: Float32) -> None:
        """
        Callback function for the power sense module (PSM) voltage topic.

        This function is called whenever a new message is received on the
        "/auv/power_sense_module/voltage" topic. It updates the internal
        state with the latest voltage data.

        Args:
            msg (std_msgs.msg.Float32): The message containing the voltage data.
        """
        self.psm_voltage_data = msg.data

    def pressure_callback(self, msg: Float32) -> None:
        """
        Callback function for the internal pressure topic.

        This function is called whenever a new message is received on the
        "/auv/pressure" topic. It updates the internal state with the latest
        pressure data.

        Args:
            msg (std_msgs.msg.Float32): The message containing the pressure data.
        """
        self.pressure_data = msg.data

    def temperature_callback(self, msg: Float32) -> None:
        """
        Callback function for the ambient temperature topic.

        This function is called whenever a new message is received on the
        "/auv/temperature" topic. It updates the internal state with the latest
        temperature data.

        Args:
            msg (std_msgs.msg.Float32): The message containing the temperature data.
        """
        self.temperature_data = msg.data

    def thruster_forces_callback(self, msg: ThrusterForces) -> None:
        """
        Callback function for the thruster forces topic.

        This function is called whenever a new message is received on the
        "/thrust/thruster_forces" topic. It updates the internal state with the
        latest thruster forces data.

        Args:
            msg (vortex_msgs.msg.ThrusterForces): The message containing the thruster forces data.
        """
        self.thruster_forces_data = msg.thrust

    def pwm_callback(self, msg: Int16MultiArray) -> None:
        """
        Callback function for the PWM signals topic.

        This function is called whenever a new message is received on the
        "/pwm" topic. It updates the internal state with the latest PWM signals data.

        Args:
            msg (std_msgs.msg.Int16MultiArray): The message containing the PWM signals data.
        """
        self.pwm_data = msg.data

    def logger(self) -> None:
        """
        Logs various sensor and actuator data to a CSV file.

        This method collects data from multiple sensors and actuators, including
        power system module (PSM) current and voltage, internal pressure, ambient
        temperature, thruster forces, and pulse-width modulation (PWM) signals.
        It then logs this data to a CSV file using the `log_data_to_csv_file` method
        of the `blackbox_log_data` object.

        The data logged includes:
        - psm_current: Current data from the power system module.
        - psm_voltage: Voltage data from the power system module.
        - pressure_internal: Internal pressure data.
        - temperature_ambient: Ambient temperature data.
        - thruster_forces_1 to thruster_forces_8: Forces data from eight thrusters.
        - pwm_1 to pwm_8: PWM signal data for eight channels.
        """
        self.blackbox_log_data.log_data_to_csv_file(
            psm_current=self.psm_current_data,
            psm_voltage=self.psm_voltage_data,
            pressure_internal=self.pressure_data,
            temperature_ambient=self.temperature_data,
            thruster_forces_1=self.thruster_forces_data[0],
            thruster_forces_2=self.thruster_forces_data[1],
            thruster_forces_3=self.thruster_forces_data[2],
            thruster_forces_4=self.thruster_forces_data[3],
            thruster_forces_5=self.thruster_forces_data[4],
            thruster_forces_6=self.thruster_forces_data[5],
            thruster_forces_7=self.thruster_forces_data[6],
            thruster_forces_8=self.thruster_forces_data[7],
            pwm_1=self.pwm_data[0],
            pwm_2=self.pwm_data[1],
            pwm_3=self.pwm_data[2],
            pwm_4=self.pwm_data[3],
            pwm_5=self.pwm_data[4],
            pwm_6=self.pwm_data[5],
            pwm_7=self.pwm_data[6],
            pwm_8=self.pwm_data[7],
        )


def main(args: list = None) -> None:
    """
    Entry point for the blackbox_node.

    This function initializes the ROS2 environment, starts the BlackBoxNode,
    and keeps it spinning until ROS2 is shut down. Once ROS2 ends, it destroys
    the node and shuts down the ROS2 environment.

    Args:
        args (list, optional): Command-line arguments passed to the ROS2 initialization.
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Start ROS2 node
    blackbox_node = BlackBoxNode()
    rclpy.spin(blackbox_node)

    # Destroy the node once ROS2 ends
    blackbox_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
