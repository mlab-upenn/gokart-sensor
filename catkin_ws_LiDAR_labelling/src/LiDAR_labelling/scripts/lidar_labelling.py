#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from pypcd import pypcd

def callback(msg):
    rospy.loginfo("I am receiving a new message with timestamp: %s.%s", msg.header.stamp.secs, msg.header.stamp.nsecs)

    # define datapath for txt and npy files
    datapath = "/home/felix/gokart-sensor/03_LiDAR_Labelling_npy/"

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
        bin_path = datapath + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".bin"
        # save as bin
        arr.astype('float32').tofile(bin_path)


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lidar_0/m1600/pcl2", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

