#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from pypcd import pypcd
import time

# define datapath for npy files
datapath_npy = "/home/felix/gokart-sensor/01_LiDAR_Dataset/npy/"

# define datapath for bin files
datapath_bin = "/home/felix/gokart-sensor/01_LiDAR_Dataset/bin/"

def callback(msg):
    if msg.header.stamp.nsecs < 50000000:
        start_time = time.time()
        rospy.loginfo("I am receiving a new message with timestamp: %s.%s", msg.header.stamp.secs, msg.header.stamp.nsecs)
    
        # convert pointcloud to binary file
        points_pcd = pypcd.PointCloud.from_msg(msg)
        x = points_pcd.pc_data['x']
        y = points_pcd.pc_data['y']
        z = points_pcd.pc_data['z']
        intensity = points_pcd.pc_data['intensity']
    
    
        npy_array = np.empty((x.shape[0], 4), dtype=np.float64)
    
        for i in range(len(x)):
            temp_array = np.array([x[i], y[i], z[i], float(intensity[i])])
            npy_array[i] = temp_array
    
        np.save(datapath_npy + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs), npy_array)
    
        arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
        arr[::4] = x
        arr[1::4] = y
        arr[2::4] = z
        arr[3::4] = intensity
    
        bin_data_path = datapath_bin + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".bin"
    
        arr.astype('float32').tofile(bin_data_path)



        print("--- %s seconds ---" % (time.time() - start_time))

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lidar_0/m1600/pcl2", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

