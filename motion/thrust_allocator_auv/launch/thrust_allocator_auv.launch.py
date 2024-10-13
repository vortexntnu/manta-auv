from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description for the thrust_allocator_auv_node.

    This function creates a ROS 2 launch description that includes the
    thrust_allocator_auv_node. The node is configured with parameters
    from the 'orca.yaml' file located in the 'auv_setup' package's
    'config/robots' directory. The output of the node is set to be
    displayed on the screen.

    Returns:
        LaunchDescription: A ROS 2 LaunchDescription object containing
        the thrust_allocator_auv_node.
    """
    thrust_allocator_auv_node = Node(
        package="thrust_allocator_auv",
        executable="thrust_allocator_auv_node",
        name="thrust_allocator_auv_node",
        parameters=[
            path.join(
                get_package_share_directory("auv_setup"),
                "config",
                "robots",
                "orca.yaml",
            )
        ],
        output="screen",
    )
    return LaunchDescription([thrust_allocator_auv_node])
