import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_filtering_node = Node(
        package='velocity_controller_lqr_cpp',
        executable='velocity_controller_lqr_cpp',
        name='velocity_controller_lqr',
        parameters=[os.path.join(get_package_share_directory('velocity_controller_lqr_cpp'), 'config', 'velocity_controller_lqr_cpp_config.yaml')],
        output='screen',
    )
    return LaunchDescription([image_filtering_node])
