import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    adap_backstepping_controller_node = Node(
        package='adap_backstepping_controller',
        executable='adap_backstepping_controller_node.py',
        name='adap_backstepping_controller_node',
        parameters=[
            os.path.join(
                get_package_share_directory('adap_backstepping_controller'),
                'config',
                'adap_backstepping_controller.yaml',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([adap_backstepping_controller_node])
