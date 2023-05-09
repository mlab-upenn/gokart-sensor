from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='ouster',
            description='Namespace for sample topics'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'os_sensor']
        ),

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/filtered_lidar'),
            # remappings=[('cloud_in', 'ouster/points'),
                        ('scan', 'ouster/scan')],
            parameters=[{
                # 'target_frame': None,
                # 'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': -0.4,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087, # 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 1.6,
                'range_max': 8.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
