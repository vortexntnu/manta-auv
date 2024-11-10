import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the velocity_controller_lqr node.

    This function creates a ROS 2 launch description that includes the
    velocity_controller_lqr node. The node is configured to use the
    parameters specified in the 'params_velocity_controller_lqr.yaml' file.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        velocity_controller_lqr node.
    """
    # Define the path to the parameter file
    parameter_file = os.path.join(
        get_package_share_directory("velocity_controller_lqr"),
        "config",
        "param_velocity_controller_lqr.yaml"
    )

    # Define the node
    velocity_controller_node = Node(
        package="velocity_controller_lqr",
        executable="velocity_controller_lqr_node.py",  # Ensure this matches your Python file name
        name="velocity_controller_lqr_node",
        output="screen",
        parameters=[parameter_file],
    )

    return LaunchDescription([velocity_controller_node])
