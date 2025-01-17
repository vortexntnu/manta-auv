#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray, Float64
from geometry_msgs.msg import PoseStamped, Wrench
from vortex_msgs.msg import ReferenceFilter
from nav_msgs.msg import Odometry
import numpy as np
from joystick_utils import Wired, WirelessXboxSeriesX, JoyStates, State, euler_to_quat, quat_to_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState


class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.last_button_press_time_ = 0
        self.debounce_duration_ = 0.25
        self.state_ = JoyStates.KILLSWITCH
        self.precise_manuevering_scaling_ = 1.0
        self.joystick_buttons_map_ = []

        self.current_state = State()
        self.desired_state = State()

        self.joystick_axes_map_ = []

        self.joy_subscriber_ = self.create_subscription(
            Joy, "orca/joy", self.joystick_cb, 5)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/orca/odom", self.odom_cb, qos_profile=best_effort_qos)
        self.wrench_publisher_ = self.create_publisher(
            Wrench, "thrust/wrench_input",5) 
        
        self.ref_publisher = self.create_publisher(
            ReferenceFilter, "/dp/guidance", qos_profile=best_effort_qos)

        self.gripper_pos_publisher_ = self.create_publisher(
            Float64, "orca/gripper_cmd", 10)

        self.gripper_rot_publisher_ = self.create_publisher(
            Float64, "orca/gripper_arm_cmd", 10)

        self.gripper_finger_publisher_ = self.create_publisher(
            Float64, "orca/gripper_finger_cmd", 10)

        self.gripper_state_publisher_ = self.create_publisher(
            JointState, "stonefish/servos", 10)

        self.declare_parameter('surge_scale_factor', 60.0)
        self.declare_parameter('sway_scale_factor', 60.0)
        self.declare_parameter('yaw_scale_factor', 60.0)
        self.declare_parameter('heave_scale_factor', 17.5)
        self.declare_parameter('roll_scale_factor', 30.0)
        self.declare_parameter('pitch_scale_factor', 20.0)

        self.declare_parameter('surge_scale_ref_param', 5000.0)
        self.declare_parameter('sway_scale_ref_param', 5000.0)
        self.declare_parameter('yaw_scale_ref_param', 5000.0)
        self.declare_parameter('heave_scale_ref_param', 5000.0)
        self.declare_parameter('roll_scale_ref_param', 5000.0)
        self.declare_parameter('pitch_scale_ref_param', 5000.0)
        

        self.gripper_desired_position_ = 0.0
        self.gripper_desired_rotation_ = 0.0
        self.gripper_grip_position_ = 0.0

        #Gets the scaling factors from the yaml file
        self.joystick_surge_scaling_ = self.get_parameter(
            'surge_scale_factor').value
        self.joystick_sway_scaling_ = self.get_parameter(
            'sway_scale_factor').value
        self.joystick_yaw_scaling_ = self.get_parameter(
            'yaw_scale_factor').value
        self.joystick_heave_scaling_ = self.get_parameter(
            'heave_scale_factor').value
        self.joystick_roll_scaling_ = self.get_parameter(
            'roll_scale_factor').value
        self.joystick_pitch_scaling_ = self.get_parameter(
            'pitch_scale_factor').value
        
        self.joystick_surge_param_ = self.get_parameter(
            'surge_scale_ref_param').value
        self.joystick_sway_param_ = self.get_parameter(
            'sway_scale_ref_param').value
        self.joystick_heave_param_ = self.get_parameter(
            'heave_scale_ref_param').value
        self.joystick_roll_param_ = self.get_parameter(
            'roll_scale_ref_param').value
        self.joystick_pitch_param_ = self.get_parameter(
            'pitch_scale_ref_param').value
        self.joystick_yaw_param_ = self.get_parameter(
            'yaw_scale_ref_param').value

        #Killswitch publisher
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softwareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))  #Killswitch is active

        #Operational mode publisher
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, "softwareOperationMode", 10) 
        self.current_pose = PoseStamped()

        self.get_logger().warn(f"Joystick interface initialized. Current mode: {self.state_}")

    def odom_cb(self, msg: Odometry) -> None:
        # extract pose from odometry message
        self.current_state.x = msg.pose.pose.position.x
        self.current_state.y = msg.pose.pose.position.y
        self.current_state.z = msg.pose.pose.position.z

        quat = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
        euler_angles = quat_to_euler(quat)
        self.current_state.roll = euler_angles[0]
        self.current_state.pitch = euler_angles[1]
        self.current_state.yaw = euler_angles[2]
    
    def create_reference_message(self) -> ReferenceFilter:
        reference_msg = ReferenceFilter()
        reference_msg.x = self.desired_state.x
        reference_msg.y = self.desired_state.y
        reference_msg.z = self.desired_state.z
        reference_msg.roll = self.desired_state.roll
        reference_msg.pitch = self.desired_state.pitch
        reference_msg.yaw = self.desired_state.yaw
        return reference_msg

    def create_wrench_message(self) -> Wrench:
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
        wrench_msg.force.x = self.surge
        wrench_msg.force.y = self.sway
        wrench_msg.force.z = self.heave
        wrench_msg.torque.x = self.roll
        wrench_msg.torque.y = self.pitch
        wrench_msg.torque.z = self.yaw
        return wrench_msg

    def transition_to_xbox_mode(self):
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.state_ = JoyStates.XBOX_MODE
        self.get_logger().warn("XBOX mode")


    def transition_to_reference_mode(self):
        """
        Publishes a pose message and signals that the operational mode has switched to Reference mode.
        """
        self.desired_state = State(
            x=self.current_state.x,
            y=self.current_state.y,
            z=self.current_state.z,
            roll=self.current_state.roll,
            pitch=self.current_state.pitch,
            yaw=self.current_state.yaw
        )
        reference_msg = self.create_reference_message()
        self.operational_mode_signal_publisher_.publish(String(data="Reference mode"))
        self.ref_publisher.publish(reference_msg)
        self.state_ = JoyStates.REFERENCE_MODE
        self.get_logger().warn("Reference mode")



    def transition_to_autonomous_mode(self):
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        empty_wrench_msg = Wrench()
        self.wrench_publisher_.publish(empty_wrench_msg)
        self.operational_mode_signal_publisher_.publish(
            String(data="autonomous mode"))
        self.state_ = JoyStates.AUTONOMOUS_MODE
        self.get_logger().warn("autonomous mode")

    def check_number_of_buttons(self, msg: Joy) -> None:
        """
        Checks if the controller is wireless (has 16 buttons) or wired and sets the joystick button and axis maps accordingly.
        
        Args:
            msg: A ROS message containing the joy input data.
        """        
        if len(msg.buttons) == 16:
            self.joystick_buttons_map_ = WirelessXboxSeriesX.joystick_buttons_map_
            self.joystick_axes_map_ = WirelessXboxSeriesX.joystick_axes_map_
        else:
            self.joystick_buttons_map_ = Wired.joystick_buttons_map_
            self.joystick_axes_map_ = Wired.joystick_axes_map_

    def populate_buttons_dictionary(self, msg: Joy) -> dict:
        """
        Populates a dictionary with button JoyStates from the joystick message.
        
        Args:
            msg: A ROS message containing the joy input data.
        
        Returns:
            A dictionary with button names as keys and their JoyStates as values.
        """
        buttons = {}
        for i, button_name in enumerate(self.joystick_buttons_map_):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
                # Assuming default value if button is not present
                buttons[button_name] = 0

        return buttons
    

    def populate_axes_dictionary(self, msg: Joy) -> dict:
        """
        Populates a dictionary with axis values from the joystick message.
        
        Args:
            msg: A ROS message containing the joy input data.
        
        Returns:
            A dictionary with axis names as keys and their values as values.
        """
        axes = {}
        for i, axis_name in enumerate(self.joystick_axes_map_):
            if i < len(msg.axes):
                axes[axis_name] = msg.axes[i]
            else:
            # Assuming default value if axis is not present
                axes[axis_name] = 0.0
        return axes
    def calculate_movement (self, axes: dict, left_trigger: float, right_trigger: float, left_shoulder: int, right_shoulder: int) -> None:
        """
        Calculates the movement values based on joystick input.

        Args:
            axes: A dictionary with axis names as keys and their JoyStates as values.
            left_trigger: The value of the left trigger.
            right_trigger: The value of the right trigger.
            left_shoulder: The state of the left shoulder button.
            right_shoulder: The state of the right shoulder button.
        """
        self.surge = axes.get(
            "vertical_axis_left_stick", 0.0
        ) * self.joystick_surge_scaling_

        self.sway = -axes.get(
            "horizontal_axis_left_stick", 0.0
        ) * self.joystick_sway_scaling_

        self.heave = (
            left_trigger - right_trigger
        ) * self.joystick_heave_scaling_

        self.roll = (
            right_shoulder - left_shoulder
        ) * self.joystick_roll_scaling_

        self.pitch = -axes.get(
            "vertical_axis_right_stick", 0.0
        ) * self.joystick_pitch_scaling_

        self.yaw = -axes.get(
            "horizontal_axis_right_stick", 0.0
        ) * self.joystick_yaw_scaling_

    def handle_killswitch_button(self):
        """
        Handles the software killswitch button press.

        This function performs the following actions based on the current state:
        1. If the current state is KILLSWITCH, it signals that the killswitch is not blocking,
            transitions to Xbox mode, and returns.
        2. Otherwise, it logs a message indicating that the software killswitch is active,
            signals that the killswitch is blocking, publishes a zero wrench message to stop
            the AUV, and sets the state to KILLSWITCH.

        The function ensures that the AUV stops moving when the killswitch is activated
        and allows it to resume operation when the killswitch is deactivated.
        """
        if self.state_ == JoyStates.KILLSWITCH:
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=False))
            self.transition_to_xbox_mode()
            return

        else:
            self.get_logger().warn("SW killswitch")
            self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))

            empty_wrench_msg = Wrench()
            self.wrench_publisher_.publish(empty_wrench_msg)
            self.state_ = JoyStates.KILLSWITCH
            return 

    def joystick_cb(self, msg: Joy):
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node. 
        Handles software killswitch and control mode buttons,
        and transitions between different JoyStates of operation.

        This function performs the following steps:
        1. Checks the number of buttons to set the correct joystick map.
        2. Populates dictionaries for button and axis JoyStates.
        3. Extracts specific button and axis values.
        4. Calculates movement values based on joystick input.
        5. Debounces button presses to prevent multiple triggers within a short duration.
        6. Updates the last button press time if any button is pressed.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A ROS message containing the wrench data that was sent to the thruster allocation node.
        """

        #Check the number of buttons and axes
        self.check_number_of_buttons(msg)

        #Populate the buttons and axes dictionaries
        buttons: dict = self.populate_buttons_dictionary(msg)
        axes: dict = self.populate_axes_dictionary(msg)
        current_time = self.get_clock().now().to_msg()._sec

        # Extract button values
        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        reference_mode_button = buttons.get("Y", 0)
        
        left_trigger = axes.get("RT", 0.0)
        right_trigger = axes.get("LT", 0.0)
        left_shoulder = buttons.get("LB", 0)
        right_shoulder = buttons.get("RB", 0)

        gripper_move = axes.get("dpad_vertical", 0.0)
        gripper_rotation = axes.get("dpad_horizontal", 0.0)
        gripper_grip = buttons.get("stick_button_left", 0)
        gripper_open = buttons.get("stick_button_right", 0)

        self.calculate_movement(axes, left_trigger, right_trigger, left_shoulder, right_shoulder)

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            reference_mode_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button or reference_mode_button:
            self.last_button_press_time_ = current_time

        # Toggle killswitch on and off
        if software_killswitch_button:
            self.handle_killswitch_button()
            
        if not self.state_ == JoyStates.KILLSWITCH:
            self.handle_gripper(gripper_move, gripper_rotation, gripper_grip, gripper_open)

        if self.state_ == JoyStates.XBOX_MODE:
            wrench_msg = self.create_wrench_message()
            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode()

        elif self.state_ == JoyStates.AUTONOMOUS_MODE:

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode() 

        elif self.state_ == JoyStates.REFERENCE_MODE:
            self.update_reference()
            msg = self.create_reference_message() 
            self.ref_publisher.publish(msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif xbox_control_mode_button:
                self.transition_to_xbox_mode()

    def handle_gripper(self, gripper_move: float, gripper_rotation: float, gripper_grip: bool, gripper_open: bool):
        gripper_state_msg = JointState()
        gripper_names = [
            "Orca/Shoulder_joint", "Orca/Arm_joint", "Orca/Finger_joint1",
            "Orca/Finger_joint2"
        ]
        gripper_pos = []

        gripper_msg = Float64()
        self.gripper_desired_position_ += gripper_move * 0.01
        if self.gripper_desired_position_ > .11:
            self.gripper_desired_position_ = .11
        if self.gripper_desired_position_ < -2.1:
            self.gripper_desired_position_ = -2.1

        gripper_pos.append(self.gripper_desired_position_)

        gripper_state_msg.name = gripper_names

        gripper_msg.data = self.gripper_desired_position_
        self.gripper_pos_publisher_.publish(gripper_msg)

        gripper_rot_msg = Float64()
        self.gripper_desired_rotation_ += gripper_rotation * 0.05
        gripper_rot_msg.data = self.gripper_desired_rotation_
        self.gripper_rot_publisher_.publish(gripper_rot_msg)

        gripper_finger_msg = Float64()
        if gripper_grip:
            self.gripper_grip_position_ += 0.01
        if gripper_open:
            self.gripper_grip_position_ -= 0.01
        if self.gripper_grip_position_ < 0:
            self.gripper_grip_position_ = 0.
        elif self.gripper_grip_position_ > 0.78:
            self.gripper_grip_position_ = 0.78

        gripper_pos.append(self.gripper_desired_rotation_)
        gripper_pos.append(self.gripper_grip_position_)
        gripper_pos.append(self.gripper_grip_position_)

        gripper_finger_msg.data = self.gripper_grip_position_
        self.gripper_finger_publisher_.publish(gripper_finger_msg)

        gripper_state_msg.position = gripper_pos

        self.gripper_state_publisher_.publish(gripper_state_msg)

    def update_reference(self): 
        """
        Updates the current pose of the AUV based on joystick inputs.
        The position and orientation (roll, pitch, yaw) are updated
        using the current joystick inputs scaled by their respective parameters.
        """
        self.desired_state.x += self.surge / self.joystick_surge_param_
        self.desired_state.y += self.sway / self.joystick_sway_param_
        self.desired_state.z += self.heave / self.joystick_heave_param_
        self.desired_state.roll += self.roll / self.joystick_roll_param_
        self.desired_state.pitch += self.pitch / self.joystick_pitch_param_
        self.desired_state.yaw += self.yaw / self.joystick_yaw_param_

        self.desired_state.roll = self.ssa(self.desired_state.roll)
        self.desired_state.pitch = self.ssa(self.desired_state.pitch)
        self.desired_state.yaw = self.ssa(self.desired_state.yaw)
    
    @staticmethod
    def ssa(angle: float) -> float:
        """
        Converts an angle from degrees to radians.

        Args:
        angle (float): The angle in degrees.

        Returns:
        float: The angle in radians.
        """
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()