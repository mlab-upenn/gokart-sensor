from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ekf_gnss'),
        'config',
        'config.yaml'
        )

    ekf_gnss_node = Node(
        package="ekf_gnss",
        executable="ekf_gnss_node",
        parameters=[config]
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     # arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    # )

    ld.add_action(ekf_gnss_node)
    return ld
