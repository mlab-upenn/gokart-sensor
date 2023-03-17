from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml


serial_port = "/dev/ttyUSB0"
frame_id = "imu_link"
operation_mode = "NDOF_FMC_OFF"
oscillator = "EXTERNAL"
reset_orientation = True
frequency = 50
use_magnetometer = False
use_temperature = False

config = [{
    "serial_port": serial_port,
    "frame_id": frame_id,
    "operation_mode": operation_mode,
    "oscillator": oscillator,
    "reset_orientation":reset_orientation,
    "frequency": frequency,
    "use_magnetometer": use_magnetometer,
    "use_temperature": use_temperature
}]

def generate_launch_description():
    ld = LaunchDescription()

    ros_imu_bno055_node = Node(
        package='ros_imu_bno055',
        executable='imu_ros',
        name='ros_imu_bno055_node',
        parameters=config
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # arguments=['-d', os.path.join(get_package_share_directory('ros_imu_bno055'), 'launch', 'view_imu_rviz.rviz')]
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link1_broadcaster',
        arguments=[f"0 0 0 0 0 0 1 fixed_frame {frame_id}"]
    )

    # finalize
    ld.add_action(ros_imu_bno055_node)
    ld.add_action(rviz_node)
    ld.add_action(tf2_node)

    return ld