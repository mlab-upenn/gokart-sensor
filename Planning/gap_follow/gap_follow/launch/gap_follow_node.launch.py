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

    ld.add_action(pointcloud_to_scan_launch)
    ld.add_action(gap_follow_node)

    return ld
