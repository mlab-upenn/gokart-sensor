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

############ fixed settings of config yaml name ###########
PUREPURSUIT_YAML_NAME = 'gnss_waypoints_purepursuit.yaml'
WPCOLLECTION_YAML_NAME = 'gnss_waypoints_collection.yaml'
GAPFOLLOW_YAML_NAME = 'ouster_2d_gap_follow.yaml'
WP_FILE_NAME = 'wp.csv'
MAP_ORI_FILE_NAME = 'map_ori.csv'
DEBUG = True
############ fixed settings of config yaml name ###########


def generate_yaml(config_folder, global_cfg_path):
    purepursuit_yaml_path = os.path.join(config_folder, PUREPURSUIT_YAML_NAME)
    wpcollection_yaml_path = os.path.join(config_folder, WPCOLLECTION_YAML_NAME)
    gapfollow_yaml_path = os.path.join(config_folder, GAPFOLLOW_YAML_NAME)
    
    with open(global_cfg_path, 'r') as f:
        global_cfg = yaml.load(f, Loader=yaml.FullLoader)
    location = global_cfg["location"]
    projection_center_latitude = global_cfg["projection_center_latitude"]
    projection_center_longitude = global_cfg["projection_center_longitude"]
    if not os.path.exists(os.path.join(config_folder, location)):
        os.makedirs(os.path.join(config_folder, location))

    gnss_to_local_map_ori_path = os.path.join(config_folder, location, MAP_ORI_FILE_NAME)

    with open(purepursuit_yaml_path, 'r') as file:
        purepursuit_yaml = yaml.load(file, Loader=yaml.FullLoader)
    with open(wpcollection_yaml_path, 'r') as file:
        wpcollection_yaml = yaml.load(file, Loader=yaml.FullLoader)
    with open(gapfollow_yaml_path, 'r') as file:
        gapfollow_yaml = yaml.load(file, Loader=yaml.FullLoader)
    
    
    purepursuit_yaml['purepursuit_node']['ros__parameters']['config_path'] = os.path.join(config_folder, location)
    purepursuit_yaml['purepursuit_node']['ros__parameters']['debug_mode'] = DEBUG
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['map_ori_path'] = gnss_to_local_map_ori_path
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['debug_mode'] = DEBUG
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['projection_center_latitude'] = projection_center_latitude
    purepursuit_yaml['gnss_to_local_node']['ros__parameters']['projection_center_longitude'] = projection_center_longitude
    purepursuit_yaml['visualize_node']['ros__parameters']['config_path'] = os.path.join(config_folder, location)

    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['map_ori_path'] = gnss_to_local_map_ori_path
    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['debug_mode'] = DEBUG
    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['projection_center_latitude'] = projection_center_latitude
    wpcollection_yaml['gnss_to_local_node']['ros__parameters']['projection_center_longitude'] = projection_center_longitude


    with open(os.path.join(config_folder, location, PUREPURSUIT_YAML_NAME), 'w') as file:
        yaml.dump(purepursuit_yaml, file)
    with open(os.path.join(config_folder, location, WPCOLLECTION_YAML_NAME), 'w') as file:
        yaml.dump(wpcollection_yaml, file)
    with open(os.path.join(config_folder, location, GAPFOLLOW_YAML_NAME), 'w') as file:
        yaml.dump(gapfollow_yaml, file)


if __name__ == "__main__":
    cwd = os.getcwd()
    global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
    config_folder = os.path.join(cwd, 'src', 'gokart-sensor', 'configs')
    print(f'current config path for all the yaml files: {config_folder}')
    generate_yaml(config_folder, global_cfg_path)