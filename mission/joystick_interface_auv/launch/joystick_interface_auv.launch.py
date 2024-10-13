import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the joystick_interface_auv node.

    This function creates a ROS 2 launch description that includes the
    joystick_interface_auv node. The node is configured to use the
    parameters specified in the 'param_joystick_interface_auv.yaml' file.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        joystick_interface_auv node.
    """
    joystick_interface_node = Node(
        package="joystick_interface_auv",
        executable="joystick_interface_auv.py",
        name="joystick_interface_auv",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("joystick_interface_auv"),
                "config/param_joystick_interface_auv.yaml",
            )
        ],
    )

    return LaunchDescription([joystick_interface_node])
