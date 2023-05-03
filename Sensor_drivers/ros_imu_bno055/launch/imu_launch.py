from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

###### get config folder path from global config file #####
cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]
config_folder = os.path.join(cwd, "src", "gokart-sensor", "configs", LOCATION)
###### get config folder path from global config file #####


serial_port = "/dev/ttyUSB0"
frame_id = "imu_link"
operation_mode = "NDOF"
oscillator = "EXTERNAL"
reset_orientation = True
frequency = 100
use_magnetometer = True
use_temperature = False

config = [{
    "serial_port": serial_port,
    "frame_id": frame_id,
    "operation_mode": operation_mode,
    "oscillator": oscillator,
    "reset_orientation":reset_orientation,
    "frequency": frequency,
    "use_magnetometer": use_magnetometer,
    "use_temperature": use_temperature,
    "config_folder": config_folder
}]

def generate_launch_description():
    ld = LaunchDescription()

    ros_imu_bno055_node = Node(
        package='ros_imu_bno055',
        executable='imu_ros',
        name='ros_imu_bno055_node',
        parameters=config
    )
    # finalize
    ld.add_action(ros_imu_bno055_node)

    return ld
