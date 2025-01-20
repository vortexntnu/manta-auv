from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

adapt_params = path.join(
    get_package_share_directory("dp_adapt_backs_controller"),
    "config",
    "adapt_params.yaml",
)


def generate_launch_description():
    dp_adapt_backs_controller_node = Node(
        package="dp_adapt_backs_controller",
        executable="dp_adapt_backs_controller_node",
        name="dp_adapt_backs_controller_node",
        parameters=[
            adapt_params,
        ],
        output="screen",
    )
    return LaunchDescription([dp_adapt_backs_controller_node])
