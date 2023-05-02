import os
import csv
from turtle import ycor
from unicodedata import name
import fire
import time
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

import numpy.linalg as LA
# import seaborn as sns
import cv2
import yaml


############ global settings ###########
PUREPURSUIT_YAML_NAME = 'gnss_waypoints_purepursuit.yaml'
WPCOLLECTION_YAML_NAME = 'gnss_waypoints_collection.yaml'
WP_FILE_NAME = 'wp.csv'
MAP_ORI_FILE_NAME = 'map_ori.csv'
DEBUG = True
LOCATION = 'pennovation'
############ global settings ###########


def PJcurvature(x, y):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    t_a = LA.norm([x[1] - x[0], y[1] - y[0]])
    t_b = LA.norm([x[2] - x[1], y[2] - y[1]])

    M = np.array([
        [1, -t_a, t_a ** 2],
        [1, 0, 0],
        [1, t_b, t_b ** 2]
    ])

    a = np.matmul(LA.inv(M), x)
    b = np.matmul(LA.inv(M), y)

    kappa = 2 * (a[2] * b[1] - b[2] * a[1]) / (a[1] ** 2. + b[1] ** 2.) ** (1.5)
    return kappa, [b[1], -a[1]] / np.sqrt(a[1] ** 2. + b[1] ** 2.)


def show_wp(config_folder, location):
    wp_path = os.path.join(config_folder, location, WP_FILE_NAME)
    waypoints = np.loadtxt(wp_path, delimiter=',', skiprows=1)
    wp_x = waypoints[:, 0]
    wp_y = waypoints[:, 1]
    print(f"waypoints: {waypoints}, shape: {waypoints.shape}")
    plt.axis('equal')
    plt.plot(wp_x, wp_y, '-ro', markersize=0.1, label='waypoints')
    plt.legend()
    plt.show()

def read_wp(config_folder, location):
    wp_path = os.path.join(config_folder, location, WP_FILE_NAME)
    waypoints = np.loadtxt(wp_path, delimiter=',', skiprows=0)
    wp_x = waypoints[:, 0]
    wp_y = waypoints[:, 1]
    return wp_x, wp_y


def visualize_curvature_for_wp(config_folder, location, ka_thres=0.05, corner_thres = 0.2):
    wp_x, wp_y = read_wp(config_folder, location)
    kappa = []
    no = []
    po = []
    ka = []
    for idx in range(len(wp_y))[1:-2]:
        x = wp_x[idx - 1:idx + 2]
        y = wp_y[idx - 1:idx + 2]
        kappa, norm = PJcurvature(x, y)
        ka.append(kappa)
        no.append(norm)
        po.append([x[1], y[1]])

    po = np.array([[wp_x[0], wp_y[1]]]+po+[[wp_x[-2], wp_y[-2]]]+[[wp_x[-1], wp_y[-1]]])
    no = np.array([no[0]]+no+[no[-2]]+[no[-1]])
    ka = np.array([ka[0]]+ka+[ka[-2]]+[ka[-1]])

    # overtaking
    i = 0
    n_ka = len(ka)
    segments = []
    while i < n_ka:
        if abs(ka[i]) < ka_thres:
            begin = i
            while i < n_ka and abs(ka[i]) < ka_thres:
                i += 1
            if i-begin > 20:
                segments.append((begin, i-1))
        else:
            i += 1
    overtaking_idx = []
    for seg in segments:
        b, e = seg[0], seg[1]
        i = b
        while i <= e:
            overtaking_idx.append(i)
            i += 1
    print("overtaking idx: ")
    print(overtaking_idx)
    idx_path = os.path.join(config_folder, LOCATION, 'overtaking_wp_idx')
    np.save(idx_path, np.array(overtaking_idx))

    # corner
    i = 0
    n_ka = len(ka)
    segments = []
    while i < n_ka:
        if abs(ka[i]) > corner_thres:
            begin = i
            while i < n_ka and abs(ka[i]) > corner_thres:
                i += 1
            if i-begin > 1:
                segments.append((begin, i-1))
        else:
            i += 1
    corner_idx = []
    for seg in segments:
        b, e = seg[0], seg[1]
        i = b
        while i <= e:
            corner_idx.append(i)
            i += 1
    print("corner idx: ")
    print(corner_idx)
    idx_path = os.path.join(config_folder, LOCATION, 'corner_wp_idx')
    np.save(idx_path, np.array(corner_idx))


    fig = plt.figure(figsize=(8, 5), dpi=120)
    ax = fig.add_subplot(2, 1, 1)
    for i in overtaking_idx:
        plt.scatter(po[i, 0], po[i, 1], c='r')
    for i in corner_idx:
        plt.scatter(po[i, 0], po[i, 1], c='b')
    plt.plot(po[:, 0], po[:, 1])
    plt.quiver(po[:, 0], po[:, 1], ka * no[:, 0], ka * no[:, 1])
    plt.axis('equal')

    ax = fig.add_subplot(2, 1, 2)
    plt.plot(ka, '-bo', markersize=0.1)
    plt.show()



if __name__ == "__main__":
    cmd = os.getcwd()
    config_folder = os.path.join(cmd, 'src', 'gokart-sensor', 'configs')
    # visualize_curvature_for_wp(config_folder, LOCATION)
    show_wp(config_folder, LOCATION)
