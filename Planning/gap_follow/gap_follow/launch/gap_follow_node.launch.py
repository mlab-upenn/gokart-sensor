from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(cwd, "src", "gokart-sensor", "configs", LOCATION, "ouster_2d_gap_follow.yaml")
    print(f"load config from {config}")
    # include another launch file
    pointcloud_to_scan_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pointcloud_to_laserscan'),
                'launch/ousterpointcloud_to_scan.launch.py'))
    )

    gap_follow_node = Node(
        package='gap_follow',
        executable='gap_follow_node',
        name='gap_follow_node',
        output='screen',
        parameters=[config]
    )

    lidar_filter_node = Node(
        package="data_aug_ros2",
        executable="lidar_filter",
        output='screen',
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'os_sensor']
    )

    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/filtered_lidar'),
                    ('scan', 'ouster/scan')],
        parameters=[config],
        name='pointcloud_to_laserscan'
    )

    obstacle_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('opponent_predictor'),
            'launch/opponent_predictor_launch.py')
    ))

    # ld.add_action(obstacle_detection_launch)
    # ld.add_action(pointcloud_to_scan_launch)
    ld.add_action(pointcloud_to_scan_node)
    ld.add_action(tf_node)
    ld.add_action(lidar_filter_node)
    ld.add_action(gap_follow_node)

    return ld
