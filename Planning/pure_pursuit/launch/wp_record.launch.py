from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
cwd = os.getcwd()

# (70, 50)-(80, 70)
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'config.yaml'
        )
    wp_path = os.path.join(
        cwd,
        'wp.csv'
        )
    print(f"wp_path: {wp_path}")
    python_node = Node(
        package="pure_pursuit",
        executable="wp_record_node",
        output="screen",
        parameters=[config, {"wp_path": wp_path}],
    )


    ld.add_action(python_node)


    
    return ld
