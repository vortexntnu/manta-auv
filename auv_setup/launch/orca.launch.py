import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the ORCA AUV setup.

    This function sets up the environment variable for ROS console formatting
    and includes the launch descriptions for the thruster allocator and thruster
    interface components of the AUV.

    Returns:
        LaunchDescription: A launch description containing the environment variable
        setting and the included launch descriptions for the thruster allocator and
        thruster interface.
    """
    # Set environment variable
    set_env_var = SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity}] [${time}] [${node}]: ${message}")

    # Thruster Allocator launch
    thrust_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thrust_allocator_auv"),
                "launch",
                "thrust_allocator_auv.launch.py",
            )
        )
    )

    # Thruster Interface launch
    thruster_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thruster_interface_auv"),
                "launch",
                "thruster_interface_auv.launch.py",
            )
        )
    )

    # Return launch description
    return LaunchDescription([set_env_var, thrust_allocator_launch, thruster_interface_launch])
