import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('guidance_los')
    config_file = os.path.join(pkg_share, 'config', 'los_guidance_params.yaml')

    guidance_node = Node(
        package='guidance_los',
        executable='los_guidance_action_server',
        name='los_guidance_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        respawn=False,
    )

    return LaunchDescription([guidance_node])
