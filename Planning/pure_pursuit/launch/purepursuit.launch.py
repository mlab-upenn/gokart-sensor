import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
# import yaml
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
        package="data_aug_ros2",
        executable="imu_listener",
        output='screen',
        parameters=[config],
    )

    # map_path = os.path.join(
    #     get_package_share_directory('pure_pursuit'),
    #     'config',
    #     'map.yaml'
    # )

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     parameters=[{'yaml_filename': map_path},
    #                 {'topic': '/map'},
    #                 {'frame_id': '/map'},
    #                 {'output': 'screen'},
    #                 {'use_sim_time': True}]
    # )

    # nav_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )

    # tf_node = Node(
    #         package='tf2_ros', 
    #         executable='static_transform_publisher',
    #         name='base_transform',
	#         output='screen',
    #         parameters = ["0.0", "0.0", "0.0", "0", "0", "0", "map","world"]
    #        ),
    
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     # arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    # )

    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    # ld.add_action(rviz_node)
    # ld.add_action(tf_node)

    ld.add_action(ekf_gnss_node)
    ld.add_action(gnss_local)
    ld.add_action(data_aug)

    ld.add_action(purepursuit_node)
    ld.add_action(visualize_node)

    
    return ld
