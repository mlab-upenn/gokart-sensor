from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('gnss_to_local'),
        'config',
        'config.yaml'
        )

    python_node = Node(
        package="gnss_to_local",
        executable="node",
        parameters=[config]
    )

    ld.add_action(python_node)
    return ld
