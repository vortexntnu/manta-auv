import rclpy
from joystick_interface_auv.joystick_interface_auv import JoystickInterface, States
from sensor_msgs.msg import Joy


class TestJoystickInterface:
    # test that the wrench msg is created successfully
    def test_wrench_msg(self) -> None:
        """
        Test the creation of a Wrench message using the JoystickInterface.

        This test initializes the ROS 2 client library, creates a Wrench message
        with specified force and torque values using the JoystickInterface, and
        asserts that the message fields are correctly set. Finally, it shuts down
        the ROS 2 client library.

        Assertions:
            - The force.x field of the message is set to 2.0.
            - The force.y field of the message is set to 3.0.
            - The force.z field of the message is set to 4.0.
            - The torque.x field of the message is set to 5.0.
            - The torque.y field of the message is set to 6.0.
            - The torque.z field of the message is set to 7.0.
        """
        rclpy.init()
        msg = JoystickInterface().create_wrench_message(2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.force.z == 4.0
        assert msg.torque.x == 5.0
        assert msg.torque.y == 6.0
        assert msg.torque.z == 7.0
        rclpy.shutdown()

    # Test that the callback function will be able to interpret the joy msg
    def test_input_from_controller_into_wrench_msg(self) -> None:
        """
        Test the conversion of joystick input to wrench message.

        This test initializes the ROS 2 client library, creates a Joy message with
        specific axes and button values, and verifies that the joystick callback
        function of the JoystickInterface class correctly converts the joystick
        input into a Wrench message with the expected force and torque values.

        Assertions:
            - The x component of the force in the wrench message should be the
              negative of the first axis value scaled by joystick_surge_scaling_.
            - The y component of the force in the wrench message should be the
              second axis value scaled by joystick_sway_scaling_.
            - The z component of the force in the wrench message should be 0.0.
            - The x, y, and z components of the torque in the wrench message should
              all be 0.0.

        Note:
            This test requires the rclpy library and the Joy and JoystickInterface
            classes to be properly imported and available in the test environment.
        """
        rclpy.init()
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = JoystickInterface().joystick_cb(joy_msg)
        assert wrench_msg.force.x == -1.0 * JoystickInterface().joystick_surge_scaling_
        assert wrench_msg.force.y == 1.0 * JoystickInterface().joystick_sway_scaling_
        assert wrench_msg.force.z == 0.0
        assert wrench_msg.torque.x == 0.0
        assert wrench_msg.torque.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    # When the killswitch button is activated in the buttons list, it should output a wrench msg with only zeros
    def test_killswitch_button(self) -> None:
        """
        Test the killswitch button functionality of the JoystickInterface.

        This test initializes the ROS 2 client library, creates an instance of the
        JoystickInterface, and sets its state to XBOX_MODE. It then creates a Joy
        message with specific axes and button values to simulate pressing the
        killswitch button. The joystick callback is invoked with this Joy message,
        and the resulting wrench message is checked to ensure that all force and
        torque components are zero, indicating that the killswitch has been
        activated. Finally, the ROS 2 client library is shut down.

        Assertions:
            - wrench_msg.force.x == 0.0
            - wrench_msg.force.y == 0.0
            - wrench_msg.force.z == 0.0
            - wrench_msg.torque.x == 0.0
            - wrench_msg.torque.y == 0.0
            - wrench_msg.torque.z == 0.0
        """
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == 0.0
        assert wrench_msg.force.y == 0.0
        assert wrench_msg.force.z == 0.0
        assert wrench_msg.torque.x == 0.0
        assert wrench_msg.torque.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    # When we move into XBOX mode it should still be able to return this wrench message
    def test_moving_in_of_xbox_mode(self) -> None:
        """
        Test the joystick callback function in XBOX mode.

        This test initializes the ROS 2 client library, creates an instance of the
        JoystickInterface, and sets its state to XBOX_MODE. It then creates a Joy
        message with specific axes and button values to simulate joystick input.
        The joystick callback function is called with this Joy message, and the
        resulting wrench message is checked to ensure that the force and torque
        values are correctly calculated based on the joystick input and scaling
        factors.

        Assertions:
            - The x-component of the force should be the negative surge scaling factor.
            - The y-component of the force should be the positive sway scaling factor.
            - The z-component of the force should be zero.
            - The x-component of the torque should be zero.
            - The y-component of the torque should be zero.
            - The z-component of the torque should be zero.
        """
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == -1.0 * JoystickInterface().joystick_surge_scaling_
        assert wrench_msg.force.y == 1.0 * JoystickInterface().joystick_sway_scaling_
        assert wrench_msg.force.z == 0.0
        assert wrench_msg.torque.x == 0.0
        assert wrench_msg.torque.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()
