from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]
cwd = os.getcwd()

def generate_launch_description():

    ld = LaunchDescription()
    config = os.path.join(cwd, "src", "gokart-sensor", "configs", LOCATION, "gnss_waypoints_collection.yaml")
    print(f"load config from {config}")
    
    wp_record_node = Node(
        package="pure_pursuit",
        executable="wp_record_node",
        output="screen",
        parameters=[config],
    )

    ekf_gnss_node = Node(
        package="ekf_gnss",
        executable="ekf_gnss_node",
        output='screen',
        parameters=[config],
    )

    gnss_local = Node(
        package="gnss_to_local",
        executable="gnss_to_local_node",
        output='screen',
        parameters=[config],
    )

    data_aug = Node(
        package="data_augment",
        executable="localization_filter",
        output='screen',
        parameters=[config],
    )

    ld.add_action(wp_record_node)
    ld.add_action(ekf_gnss_node)
    ld.add_action(gnss_local)
    ld.add_action(data_aug)
    
    return ld
