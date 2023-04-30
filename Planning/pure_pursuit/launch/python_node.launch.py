from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

cwd = os.getcwd()

# (70, 50)-(80, 70)
def generate_launch_description():
    ld = LaunchDescription([yaml_path])
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
    map_path = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'map.yaml'
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

    ld.add_action(python_node)
    ld.add_action(visualize_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    # ld.add_action(rviz_node)
    # ld.add_action(tf_node)

    
    return ld
