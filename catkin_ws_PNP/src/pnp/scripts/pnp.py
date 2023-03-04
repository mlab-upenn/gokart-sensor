#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
import time

pub = rospy.Publisher('/marker', MarkerArray, queue_size=10)

def callback(msg):
    bb_arr = msg.bounding_boxes
    i = 0

    global average_time
    global average_time_counter
    time_start = time.time()
    average_time_counter += 1

    marker_array = MarkerArray()

    # deletes all previous markers
    marker = Marker()
    marker.header.frame_id = "os_sensor"
    marker.id = 0
    marker.ns = "detections"
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)
    pub.publish(marker_array)

    marker_array = MarkerArray()

    for bs in bb_arr:

        # calculate position of center and height of bounding box
        x_center_pixel = (bs.xmax - bs.xmin) * 0.5 + bs.xmin
        y_center_pixel = (bs.ymax - bs.ymin) * 0.5 + bs.ymin
        height_pixel = (bs.ymax - bs.ymin)

        # cone paramter
        height_meter = 0.7

        # calibration matrix from LiDAR to camera frame
        l_t_c = np.array([[0.0190983, -0.999815, 0.00232702, 0.0440449],
                          [-0.193816, -0.0059855, -0.98102, 0.142687],
                          [0.980852, 0.0182848, -0.193894, -0.0378563],
                          [0, 0, 0, 1]])

        # intrinsic camera parameters
        fx = 1442.193090
        fy = 1440.905704
        cx = 943.936435
        cy = 542.578706

        # calculate x, y, and z position in camera frame
        Z_meter_cam = (fy * height_meter) / height_pixel
        X_meter_cam = ((x_center_pixel - cx) * Z_meter_cam) / fx
        Y_meter_cam = ((y_center_pixel - cy) * Z_meter_cam) / fy

        # convert cartesian coordinates in homogenous coordinates and transform to LiDAR frame
        point_meter_cam = np.array([X_meter_cam, Y_meter_cam, Z_meter_cam, 1])
        point_meter_lidar = np.linalg.inv(l_t_c) @ point_meter_cam

        # publish detections as markers for RViZ
        marker = Marker()
        marker.header.frame_id = "os_sensor"
        marker.header.stamp = rospy.Time.now()
        marker.action = 0
        marker.type = 3
        marker.id = i
        marker.color.a = 0.4
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.scale.x = 0.32
        marker.scale.y = 0.32
        marker.scale.z = 0.7
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.pose.position.x = point_meter_lidar[0]
        marker.pose.position.y = point_meter_lidar[1]
        marker.pose.position.z = point_meter_lidar[2] + 0.7
        i += 1
        marker_array.markers.append(marker)

    # to print time per iteration, uncomment next line
    print("Time elapsed: ", time.time() - time_start, " seconds")
    average_time += time.time() - time_start
    print("Average time elapsed: ", average_time / average_time_counter, " seconds")
    pub.publish(marker_array)



def listener():
    global average_time
    global average_time_counter
    average_time = 0
    average_time_counter = 0

    rospy.init_node('pnp', anonymous=True)

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

