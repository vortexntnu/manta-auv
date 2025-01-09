import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

config_file_path = os.path.join(
    get_package_share_directory('reference_filter_dp'),
    'config',
    'reference_filter_params.yaml',
)


def generate_launch_description():
    reference_filter_node = Node(
        package='reference_filter_dp',
        executable='reference_filter_node',
        name='reference_filter_node',
        parameters=[
            config_file_path,
        ],
        output='screen',
    )
    return LaunchDescription([reference_filter_node])
