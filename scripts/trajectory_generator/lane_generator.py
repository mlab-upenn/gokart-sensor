import numpy as np
import time
import json
import os
import trajectory_planning_helpers as tph
import copy
import matplotlib.pyplot as plt
import configparser
import pkg_resources
import helper_funcs_glob
import yaml

###### get config folder path from global config file #####
cwd = os.getcwd()
global_cfg_path = os.path.join(cwd, "src", "gokart-sensor", "configs", "global_config.yaml")
with open(global_cfg_path, 'r') as f:
    global_cfg = yaml.load(f, Loader=yaml.FullLoader)
LOCATION = global_cfg["location"]
TRACK_SPAN = global_cfg["track_width"]/2
config_folder = os.path.join(cwd, "src", "gokart-sensor", "configs")
###### get config folder path from global config file #####
BOUND1_FILE_NAME = "bound1.csv"
BOUND2_FILE_NAME = "bound2.csv"
WP_FILE_NAME = "optim_traj.csv"
WP_DELIM = ";"
WP_SKIPROW = 3
WP_X_IDX = 1
WP_Y_IDX = 2
WP_V_IDX = 5
LANE_DEVIATION = 1.0

def lane_generator():
    traj_all = np.loadtxt(os.path.join(config_folder, LOCATION, WP_FILE_NAME), delimiter=WP_DELIM, skiprows=WP_SKIPROW)
    overtake_idx = np.load(os.path.join(config_folder, LOCATION, "overtake_wp_idx.npy"))
    traj = traj_all[:, [WP_X_IDX, WP_Y_IDX]]
    _, _, _, normvec = tph.calc_splines.calc_splines(path=traj)
    traj_noclose = traj[:-1, :]
    track_span_arr = np.ones((traj_noclose.shape[0], 1)) * TRACK_SPAN
    lane_span_arr = np.ones((traj_noclose.shape[0], 1)) * LANE_DEVIATION
    bound1 = traj_noclose + normvec * track_span_arr
    bound2 = traj_noclose - normvec * track_span_arr
    lane1 = traj_noclose + normvec * lane_span_arr
    lane2 = traj_noclose - normvec * lane_span_arr
    
    plt.plot(bound1[:, 0], bound1[:, 1], 'r', label='outer boundary')
    plt.plot(bound2[:, 0], bound2[:, 1], 'b', label='inner boundary')
    plt.plot(traj[:, 0], traj[:, 1], 'k', label='centerline')
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    lane_generator()
