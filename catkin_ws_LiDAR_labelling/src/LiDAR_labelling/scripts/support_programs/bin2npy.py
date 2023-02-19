#!/usr/bin/env python3

import os
import numpy as np

datapath_bin = "/home/felix/gokart-sensor/03_LiDAR_Dataset/custom/points"
datapath_npy = "/home/felix/gokart-sensor/03_LiDAR_Dataset/custom/points_npy"

for filename in os.listdir(datapath_bin):
    # open json file
    f_bin = datapath_bin + "/" + filename
    f_npy = datapath_npy + "/" + filename[:-3] + "npy"

    arr = np.fromfile(f_bin, dtype=np.float32)
    np.save(f_npy, arr)
