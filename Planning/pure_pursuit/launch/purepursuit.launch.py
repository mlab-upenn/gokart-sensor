import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
# from ament_index_python.packages import get_package_share_directory

cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]
OPTIMIZE = False

for arg in sys.argv:
    if arg.startswith("optimize:="):
        OPTIMIZE = eval(arg.split(":=")[1])

    if arg.startswith("location:="):
        LOCATION = eval(arg.split(":=")[1])

def generate_launch_description():
    ld = LaunchDescription()

    if OPTIMIZE:
        FILENAME = 'gnss_waypoints_purepursuit_optimal.yaml'
    else:
        FILENAME = 'gnss_waypoints_purepursuit_original.yaml'

    config = os.path.join(cwd, "src", "gokart-sensor", "configs", LOCATION, FILENAME)
    print(f"load config from {config}")

    purepursuit_node = Node(
        package="pure_pursuit",
        executable="purepursuit_node",
        output="screen",
        parameters=[config],
    )

    visualize_node = Node(
        package="pure_pursuit",
        executable="visualize_node",
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

    ld.add_action(ekf_gnss_node)
    ld.add_action(gnss_local)
    ld.add_action(data_aug)

    ld.add_action(purepursuit_node)
    ld.add_action(visualize_node)

    
    return ld
