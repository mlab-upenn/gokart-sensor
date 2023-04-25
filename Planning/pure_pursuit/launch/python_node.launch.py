from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# (70, 50)-(80, 70)
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'config.yaml'
        )
    wp_path = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'wp.csv'
        )

    python_node = Node(
        package="pure_pursuit",
        executable="python_node",
        output="screen",
        parameters=[config, {"wp_path": wp_path}],
    )

    visualize_node = Node(
        package="pure_pursuit",
        executable="visualize_node",
        output="screen",
        parameters=[config, {"wp_path": wp_path}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )

    ld.add_action(python_node)
    ld.add_action(visualize_node)
    ld.add_action(rviz_node)
    return ld
