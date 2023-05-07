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
LANE_DEVIATION = 0.7

def lane_generator():
    traj_all = np.loadtxt(os.path.join(config_folder, LOCATION, WP_FILE_NAME), delimiter=WP_DELIM, skiprows=WP_SKIPROW)
    overtake_idx = np.load(os.path.join(config_folder, LOCATION, "overtaking_wp_idx.npy"))
    overtake_idx = overtake_idx.flatten()
    traj = traj_all[:, [WP_X_IDX, WP_Y_IDX]]
    _, _, _, normvec = tph.calc_splines.calc_splines(path=traj)
    traj_noclose = traj[:-1, :]
    track_span_arr = np.ones((traj_noclose.shape[0], 1)) * TRACK_SPAN
    lane_span_arr = np.ones((traj_noclose.shape[0], 1)) * LANE_DEVIATION
    bound1 = traj_noclose + normvec * track_span_arr
    bound2 = traj_noclose - normvec * track_span_arr
    lane1 = traj_noclose - normvec * lane_span_arr
    lane2 = traj_noclose + normvec * lane_span_arr
    traj_lane1 = copy.deepcopy(traj_all[:-1, :])
    traj_lane2 = copy.deepcopy(traj_all[:-1, :])
    traj_lane1[overtake_idx, WP_X_IDX] = lane1[overtake_idx, 0]
    traj_lane1[overtake_idx, WP_Y_IDX] = lane1[overtake_idx, 1]
    traj_lane2[overtake_idx, WP_X_IDX] = lane2[overtake_idx, 0]
    traj_lane2[overtake_idx, WP_Y_IDX] = lane2[overtake_idx, 1]
    traj_lane1 = np.vstack((traj_lane1, traj_lane1[0, :]))
    traj_lane2 = np.vstack((traj_lane2, traj_lane2[0, :]))
    
    np.savetxt(os.path.join(config_folder, LOCATION, "lane1.csv"), traj_lane1, delimiter=WP_DELIM)
    np.savetxt(os.path.join(config_folder, LOCATION, "lane2.csv"), traj_lane2, delimiter=WP_DELIM)
    plt.plot(traj_lane1[:, WP_X_IDX], traj_lane1[:, WP_Y_IDX], '-ro', label='lane1', markersize=1.0)
    plt.plot(traj_lane2[:, WP_X_IDX], traj_lane2[:, WP_Y_IDX], '-bo', label='lane2', markersize=1.0)
    plt.plot(traj[:, 0], traj[:, 1], 'k', label='centerline', linewidth=0.1)
    plt.plot(bound1[:, 0], bound1[:, 1], 'k', label='bound1', linewidth=0.2)
    plt.plot(bound2[:, 0], bound2[:, 1], 'k', label='bound2', linewidth=0.2)
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    lane_generator()
