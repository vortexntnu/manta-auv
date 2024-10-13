#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class States:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3


class Wired:
    joystick_buttons_map_ = [
        "A",
        "B",
        "X",
        "Y",
        "LB",
        "RB",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick",  # Sway
        "vertical_axis_left_stick",  # Surge
        "LT",  # Negative thrust/torque multiplier
        "horizontal_axis_right_stick",  # Yaw
        "vertical_axis_right_stick",
        "RT",  # Positive thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
    ]


class WirelessXboxSeriesX:
    joystick_buttons_map_ = [
        "A",
        "B",
        "0",
        "X",
        "Y",
        "0",
        "LB",
        "RB",
        "0",
        "0",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick",  # Sway
        "vertical_axis_left_stick",  # Surge
        "horizontal_axis_right_stick",  # Yaw
        "vertical_axis_right_stick",
        "RT",  # Positive thrust/torque multiplier
        "LT",  # Negative thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
    ]


class JoystickInterface(Node):
    def __init__(self) -> None:
        super().__init__("joystick_interface_node")
        self.get_logger().info(
            "Joystick interface is up and running. \n When the XBOX controller is connected, press the killswitch button once to enter XBOX mode."
        )

        self.last_button_press_time_ = 0
        self.debounce_duration_ = 0.25
        self.state_ = States.NO_GO
        self.precise_manuevering_mode_ = False
        self.precise_manuevering_scaling_ = 1.0

        self.joystick_buttons_map_ = []

        self.joystick_axes_map_ = []

        self.joy_subscriber_ = self.create_subscription(Joy, "joystick/joy", self.joystick_cb, 5)
        self.wrench_publisher_ = self.create_publisher(Wrench, "thrust/wrench_input", 5)

        self.declare_parameter("surge_scale_factor", 60.0)
        self.declare_parameter("sway_scale_factor", 60.0)
        self.declare_parameter("yaw_scale_factor", 60.0)
        self.declare_parameter("heave_scale_factor", 17.5)
        self.declare_parameter("roll_scale_factor", 30.0)
        self.declare_parameter("pitch_scale_factor", 20.0)

        # Gets the scaling factors from the yaml file
        self.joystick_surge_scaling_ = self.get_parameter("surge_scale_factor").value
        self.joystick_sway_scaling_ = self.get_parameter("sway_scale_factor").value
        self.joystick_yaw_scaling_ = self.get_parameter("yaw_scale_factor").value
        self.joystick_heave_scaling_ = self.get_parameter("heave_scale_factor").value
        self.joystick_roll_scaling_ = self.get_parameter("roll_scale_factor").value
        self.joystick_pitch_scaling_ = self.get_parameter("pitch_scale_factor").value

        # Killswitch publisher
        self.software_killswitch_signal_publisher_ = self.create_publisher(Bool, "softwareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(Bool(data=True))  # Killswitch is active

        # Operational mode publisher
        self.operational_mode_signal_publisher_ = self.create_publisher(String, "softwareOperationMode", 10)

        # Signal that we are in XBOX mode
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))

    def create_wrench_message(
        self,
        surge: float,
        sway: float,
        heave: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Wrench:
        """
        Creates a 2D wrench message with the given x, y, heave, roll, pitch, and yaw values.

        Args:
            surge (float): The x component of the force vector.
            sway (float): The y component of the force vector.
            heave (float): The z component of the force vector.
            roll (float): The x component of the torque vector.
            pitch (float): The y component of the torque vector.
            yaw (float): The z component of the torque vector.

        Returns:
            Wrench: A 2D wrench message with the given values.
        """
        wrench_msg = Wrench()
        wrench_msg.force.x = surge
        wrench_msg.force.y = sway
        wrench_msg.force.z = heave
        wrench_msg.torque.x = roll
        wrench_msg.torque.y = pitch
        wrench_msg.torque.z = yaw
        return wrench_msg

    def transition_to_xbox_mode(self) -> None:
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.state_ = States.XBOX_MODE

    def transition_to_autonomous_mode(self) -> None:
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        wrench_msg = self.create_wrench_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.wrench_publisher_.publish(wrench_msg)
        self.operational_mode_signal_publisher_.publish(String(data="autonomous mode"))
        self.state_ = States.AUTONOMOUS_MODE

    def joystick_cb(self, msg: Joy) -> Wrench:
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node.
        Handles software killswitch and control mode buttons,
        and transitions between different states of operation.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A ROS message containing the wrench data that was sent to the thruster allocation node.
        """

        current_time = self.get_clock().now().to_msg().sec

        buttons = {}
        axes = {}

        # Check if the controller is wireless (has 16 buttons) or wired
        if len(msg.buttons) == 16:
            self.joystick_buttons_map_ = WirelessXboxSeriesX.joystick_buttons_map_
            self.joystick_axes_map_ = WirelessXboxSeriesX.joystick_axes_map_
        else:
            self.joystick_buttons_map_ = Wired.joystick_buttons_map_
            self.joystick_axes_map_ = Wired.joystick_axes_map_

        # Populate buttons dictionary
        for i, button_name in enumerate(self.joystick_buttons_map_):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
                # Assuming default value if button is not present
                buttons[button_name] = 0

        # Populate axes dictionary
        for i, axis_name in enumerate(self.joystick_axes_map_):
            if i < len(msg.axes):
                axes[axis_name] = msg.axes[i]
            else:
                # Assuming default value if axis is not present
                axes[axis_name] = 0.0

        # Extract button values
        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        precise_manuevering_mode_button = buttons.get("Y", 0)
        left_trigger = axes.get("RT", 0.0)
        right_trigger = axes.get("LT", 0.0)
        left_shoulder = buttons.get("LB", 0)
        right_shoulder = buttons.get("RB", 0)

        # Extract axis values
        surge = axes.get("vertical_axis_left_stick", 0.0) * self.joystick_surge_scaling_ * self.precise_manuevering_scaling_
        sway = -axes.get("horizontal_axis_left_stick", 0.0) * self.joystick_sway_scaling_ * self.precise_manuevering_scaling_
        heave = (left_trigger - right_trigger) * self.joystick_heave_scaling_ * self.precise_manuevering_scaling_
        roll = (right_shoulder - left_shoulder) * self.joystick_roll_scaling_ * self.precise_manuevering_scaling_
        pitch = -axes.get("vertical_axis_right_stick", 0.0) * self.joystick_pitch_scaling_ * self.precise_manuevering_scaling_
        yaw = -axes.get("horizontal_axis_right_stick", 0.0) * self.joystick_yaw_scaling_ * self.precise_manuevering_scaling_

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            precise_manuevering_mode_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button or precise_manuevering_mode_button:
            self.last_button_press_time_ = current_time

        # Toggle killswitch on and off
        if software_killswitch_button:
            if self.state_ == States.NO_GO:
                # Signal that killswitch is not blocking
                self.software_killswitch_signal_publisher_.publish(Bool(data=False))
                self.transition_to_xbox_mode()
                return

            self.get_logger().info("SW killswitch", throttle_duration_sec=1)
            # Signal that killswitch is blocking
            self.software_killswitch_signal_publisher_.publish(Bool(data=True))

            # Publish a zero wrench message when killswitch is activated
            wrench_msg = self.create_wrench_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.wrench_publisher_.publish(wrench_msg)
            self.state_ = States.NO_GO
            return wrench_msg

        # Toggle precise maneuvering mode on and off
        if precise_manuevering_mode_button:
            self.precise_manuevering_mode_ = not self.precise_manuevering_mode_
            mode = "ENABLED" if self.precise_manuevering_mode_ else "DISABLED"
            self.get_logger().info(f"Precise maneuvering mode {mode}.")
            self.precise_manuevering_scaling_ = 0.5 if self.precise_manuevering_mode_ else 1.0

        # Publish wrench message from joystick_interface to thrust allocation
        wrench_msg = self.create_wrench_message(surge, sway, heave, roll, pitch, yaw)

        if self.state_ == States.XBOX_MODE:
            self.get_logger().info("XBOX", throttle_duration_sec=1)
            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()

        if self.state_ == States.AUTONOMOUS_MODE:
            self.get_logger().info("autonomous mode", throttle_duration_sec=1)

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()

        return wrench_msg


def main() -> None:
    """
    Initializes the ROS 2 client library, creates an instance of the JoystickInterface node,
    and starts spinning the node to process callbacks. Once the node is shut down, it destroys
    the node and shuts down the ROS 2 client library.

    This function is the entry point for the joystick interface application.
    """
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
