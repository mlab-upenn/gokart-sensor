import yaml
import os

"""
This script is used to fill the yaml file for purepursuit and wpcollection under the corresponding location folder in the config folder.
It should be invoked under the root folder of the workspace, like"python3 src/gokart-sensor/scripts/generate_yaml.py"

The script will fill the following parameters in the yaml file:
gnss_to_local_node:
    map_ori_path: the path of the map_ori file
    debug_mode: whether to show the debug info

purepursuit_node:
    wp_path: the path of the waypoint file
    debug_mode: whether to show the debug info

You still need to change the control parameters such as lookahead_distance and the max_speed factor in the purepursuit yaml file manually.
"""

############ global settings ###########
PUREPURSUIT_YAML_NAME = 'gnss_waypoints_purepursuit.yaml'
WPCOLLECTION_YAML_NAME = 'gnss_waypoints_collection.yaml'
WP_FILE_NAME = 'wp.csv'
MAP_ORI_FILE_NAME = 'map_ori.csv'
DEBUG = True
LOCATION = 'pennovation'
############ global settings ###########


def generate_yaml(config_folder, location):
    purepursuit_yaml_path = os.path.join(config_folder, PUREPURSUIT_YAML_NAME)
    wpcollection_yaml_path = os.path.join(config_folder, WPCOLLECTION_YAML_NAME)

    purepursuit_wp_path = os.path.join(config_folder, location, WP_FILE_NAME)
    gnss_to_local_map_ori_path = os.path.join(config_folder, location, MAP_ORI_FILE_NAME)

    with open(purepursuit_yaml_path, 'r') as file:
        purepursuit_yaml = yaml.load(file, Loader=yaml.FullLoader)
    with open(wpcollection_yaml_path, 'r') as file:
        wpcollection_yaml = yaml.load(file, Loader=yaml.FullLoader)
    
    purepursuit_yaml['purepursuit_node']['ros__parameters']['wp_path'] = purepursuit_wp_path
    purepursuit_yaml['purepursuit_node']['ros__parameters']['debug_mode'] = DEBUG
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['map_ori_path'] = gnss_to_local_map_ori_path
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['debug_mode'] = DEBUG

    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['map_ori_path'] = gnss_to_local_map_ori_path
    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['debug_mode'] = DEBUG
    wpcollection_yaml['wp_record_node']['ros__parameters']['wp_path'] = purepursuit_wp_path

    with open(os.path.join(config_folder, location, PUREPURSUIT_YAML_NAME), 'w') as file:
        yaml.dump(purepursuit_yaml, file)
    
    with open(os.path.join(config_folder, location, WPCOLLECTION_YAML_NAME), 'w') as file:
        yaml.dump(wpcollection_yaml, file)


if __name__ == "__main__":
    cmd = os.getcwd()
    config_folder = os.path.join(cmd, 'src', 'gokart-sensor', 'configs')
    print(f'current config path for all the yaml files: {config_folder}')
    generate_yaml(config_folder, LOCATION)