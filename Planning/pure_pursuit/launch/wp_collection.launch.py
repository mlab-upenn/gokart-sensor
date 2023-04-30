from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
cwd = os.getcwd()
wp_ori_path = os.path.join(cwd, 'wp_ori.csv')

# (70, 50)-(80, 70)
def generate_launch_description():
    ld = LaunchDescription()

    ekf_gnss_node = Node(
        package="ekf_gnss",
        executable="ekf_gnss_node",
        output='screen'
    )

    gnss_local = Node(
        package="gnss_to_local",
        executable="gnss_to_local_node",
        output='screen'
    )

    data_aug = Node(
        package="data_aug_ros2",
        executable="imu_listener",
        output='screen'
    )

    ld.add_action(ekf_gnss_node)
    ld.add_action(gnss_local)
    ld.add_action(data_aug)

    
    return ld
