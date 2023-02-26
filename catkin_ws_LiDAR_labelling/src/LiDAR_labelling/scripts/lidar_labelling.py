#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from pypcd import pypcd
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import pcl
import open3d as o3d

def callback(msg):
    rospy.loginfo("I am receiving a new message with timestamp: %s.%s", msg.header.stamp.secs, msg.header.stamp.nsecs)

    # define datapath for txt and npy files
    datapath_bin = "/home/felix/gokart-sensor/01_LiDAR_Dataset/bin_test"
    datapath_npy = "/home/felix/gokart-sensor/01_LiDAR_Dataset/npy_test"
    datapath_pcd = "/home/felix/gokart-sensor/01_LiDAR_Dataset/pcd_test"

    # only save 1 bin file per second
    if msg.header.stamp.nsecs <= 50000000:
        # convert pointcloud to binary file
        points_pcd = pypcd.PointCloud.from_msg(msg)
        x = points_pcd.pc_data['x']
        y = points_pcd.pc_data['y']
        z = points_pcd.pc_data['z']
        intensity = points_pcd.pc_data['intensity']
        arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
        arr[::4] = x
        arr[1::4] = y
        arr[2::4] = z
        arr[3::4] = intensity
        # save file at the defined path with the timestamp as file name
        bin_path = datapath_bin + "/" + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".bin"
        # save as bin
        arr.astype('float32').tofile(bin_path)
        # save as npy
        np.save(datapath_npy + "/" + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".npy", arr)

        # convert array to pcd
        pc_data = ros_numpy.numpify(msg)
        pc_array = np.zeros((pc_data.shape[0], 3))
        pc_array[:, 0] = pc_data['x']
        pc_array[:, 1] = pc_data['y']
        pc_array[:, 2] = pc_data['z']
        # pcd = pcl.PointCloud(np.array(pc_array, dtype=np.float32))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_array)
        o3d.io.write_point_cloud(datapath_pcd + "/" + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".pcd", pcd)



def listener():

    rospy.init_node('lidar_labellingZ', anonymous=True)

    rospy.Subscriber("/lidar_0/m1600/pcl2", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

