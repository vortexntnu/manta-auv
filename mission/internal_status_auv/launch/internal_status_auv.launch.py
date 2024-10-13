import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a LaunchDescription object that defines the nodes to be launched.

    This function creates a launch configuration for three sensor nodes:
    Power Sense Module Node, Pressure Sensor Node, and Temperature Sensor Node.
    Each node is configured using parameters from a YAML file located in the
    auv_setup directory.

    The nodes will be launched with their respective executables and display
    output on the screen.

    Returns:
    --------
    launch.LaunchDescription
        A LaunchDescription object containing the nodes to be launched.
    """
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("internal_status_auv"),
        "../../../../",  # Navigate to the workspace root
        "src/vortex-auv/auv_setup/config/robots/",  # Directory containing YAML files
        "orca.yaml",  # Configuration file for the orca robot
    )

    # Power Sense Module Node
    power_sense_module_node = Node(
        package="internal_status_auv",
        executable="power_sense_module_node.py",
        output="screen",
        parameters=[yaml_file_path],
    )

    # Pressure Sensor Node
    pressure_sensor_node = Node(
        package="internal_status_auv",
        executable="pressure_sensor_node.py",
        output="screen",
        parameters=[yaml_file_path],
    )

    # Temperature Sensor Node
    temperature_sensor_node = Node(
        package="internal_status_auv",
        executable="temperature_sensor_node.py",
        output="screen",
        parameters=[yaml_file_path],
    )

    # Launch Nodes
    return LaunchDescription(
        [
            power_sense_module_node,
            pressure_sensor_node,
            temperature_sensor_node,
        ]
    )
