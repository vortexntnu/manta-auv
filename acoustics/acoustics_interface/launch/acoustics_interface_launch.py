import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the acoustics_interface node.

    This function constructs the path to the YAML configuration file for the
    acoustics_interface node and returns a LaunchDescription object that
    includes the node with the specified parameters.

    Returns:
        LaunchDescription: A launch description object that includes the
        acoustics_interface node with the specified parameters.
    """
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("acoustics_interface"),
        "../../../../",  # Go to the workspace
        "src/vortex-auv/auv_setup/config/robots/",  # Go inside where yamal files are located at
        "orca.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="acoustics_interface",
                namespace="acoustics_interface",
                executable="acoustics_interface_node.py",
                name="acoustics_interface_node",
                output="screen",
                parameters=[yaml_file_path],
            ),
        ]
    )
