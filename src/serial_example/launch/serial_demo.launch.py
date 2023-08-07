from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('serial_example'), 'config', 'serial_driver.yaml')

    return LaunchDescription([
        Node(
            package='serial_example',
            namespace='',
            executable='serial_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])