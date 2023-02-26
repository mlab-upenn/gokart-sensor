#!/usr/bin/env python3

import os
import numpy as np
import open3d as o3d

datapath_bin = "/home/felix/gokart-sensor/01_LiDAR_Dataset/bin"
datapath_npy = "/home/felix/gokart-sensor/01_LiDAR_Dataset/npy"
datapath_pcd = "/home/felix/gokart-sensor/01_LiDAR_Dataset/pcd"

for filename in os.listdir(datapath_bin):
    # open json file
    f_bin = datapath_bin + "/" + filename
    f_npy = datapath_npy + "/" + filename[:-3] + "npy"

    arr = np.fromfile(f_bin, dtype=np.float32)
    print(arr.shape)
    np.save(f_npy, arr)

    # convert array to pcd
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(arr[:, :3])

    # save pcd file
    o3d.io.write_point_cloud(datapath_npy + "/" + filename[:-3] + "pcd", pcd)
