#!/usr/bin/env python3
#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import ros_numpy
import math
from random import *
import time
import numpy as np

pub = rospy.Publisher('/velo_points', pc2, queue_size=10)

# Fields of new PointCloud2 including ring values
# Information about datatype of ring value from https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2
# Definition of PointField can be found here http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('ring', 12, PointField.UINT16, 1)
          ]



def callback(data):
    time_start = time.perf_counter()
    xyz_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    # data_arr = ros_numpy.point_cloud2.pointcloud2_to_array(data)   
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "lidar_0"
    xyzr = []
    over = 0
    under = 0
    alpha_max = 0
    alpha_min = 0
    max_ring = 0
    min_ring = 100
    random_point = randint(0,xyz_arr.shape[0]-1)

    for i in range(xyz_arr.shape[0]):
        distance = math.sqrt(math.pow(xyz_arr[i][0],2) + math.pow(xyz_arr[i][1],2))
        alpha = math.atan(xyz_arr[i][2] / distance)
        if alpha > alpha_max:
            alpha_max = alpha
        if alpha < alpha_min:
            alpha_min = alpha

    for i in range(xyz_arr.shape[0]):
        distance = math.sqrt(math.pow(xyz_arr[i][0],2) + math.pow(xyz_arr[i][1],2))
        alpha = math.atan(xyz_arr[i][2] / distance)
        ring_value = 15 * ((alpha_min - alpha_max) - (alpha - alpha_max)) / (alpha_min - alpha_max)
        if math.isnan(ring_value) == True:
            ring_value = 0
        if round(ring_value) >= 16:
            over += 1
        elif round(ring_value) < 16 and round(ring_value) != 0:
            under += 1
        #if round(ring_value) == 1:
            # print(ring_value)
        xyzr.append([xyz_arr[i][0],xyz_arr[i][1],xyz_arr[i][2],round(ring_value)])
        
        # if data_arr[i]['ring'] > max_ring:
        #     max_ring = data_arr[i]['ring']
        # if data_arr[i]['ring'] < min_ring:
        #     min_ring = data_arr[i]['ring']
        
        if i == random_point:
            print("XYZ: ", xyzr[i][0], xyzr[i][1], xyzr[i][2], "Ring value: ", xyzr[i][3])
            print("Min and Max Alpha: ", alpha_min, " ", alpha_max, "Alpha: ", alpha, "Distance: ", distance)
            # print("Original Ring value is:", data_arr[i]['ring'], "and the calculated ring value is: ", round(ring_value))
    vel_pc = point_cloud2.create_cloud(header, fields, xyzr)
    time_end = time.perf_counter()
    print("Time: ",time_end-time_start, "s")
    print("Min Ring: ", min_ring, "Max Ring: ", max_ring)
    pub.publish(vel_pc)


def listener():
    rospy.init_node('pcl2velodyne', anonymous=True)
    rospy.Subscriber("/ouster/points", pc2, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
