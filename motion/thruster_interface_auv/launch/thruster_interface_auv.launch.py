from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the thruster_interface_auv_node.

    This function creates a ROS 2 launch description that includes the
    thruster_interface_auv_node. The node is configured to output to the screen
    and emulate a TTY. It also loads parameters from the orca.yaml configuration
    file located in the auv_setup package.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        thruster_interface_auv_node.
    """
    thruster_interface_auv_node = Node(
        package="thruster_interface_auv",
        executable="thruster_interface_auv_node.py",
        name="thruster_interface_auv_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            path.join(
                get_package_share_directory("auv_setup"),
                "config",
                "robots",
                "orca.yaml",
            )
        ],
    )
    return LaunchDescription([thruster_interface_auv_node])
