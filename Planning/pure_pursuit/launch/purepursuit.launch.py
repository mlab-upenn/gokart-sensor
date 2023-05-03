from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

LOCATION = 'pennovation'
cwd = os.getcwd()

def generate_launch_description():

    ld = LaunchDescription()
    config = os.path.join(cwd, "src", "gokart-sensor", "configs", LOCATION, "gnss_waypoints_purepursuit.yaml")
    print(f"load config from {config}")
    map_path = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'map.yaml'
        )

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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path},
                    {'topic': '/map'},
                    {'frame_id': '/map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # tf_node = Node(
    #         package='tf2_ros', 
    #         executable='static_transform_publisher',
    #         name='base_transform',
	#         output='screen',
    #         parameters = ["0.0", "0.0", "0.0", "0", "0", "0", "map","world"]
    #        ),
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
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

    ld.add_action(ekf_gnss_node)
    ld.add_action(gnss_local)
    ld.add_action(data_aug)

    ld.add_action(purepursuit_node)
    ld.add_action(visualize_node)
    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    # ld.add_action(rviz_node)
    # ld.add_action(tf_node)

    
    return ld
