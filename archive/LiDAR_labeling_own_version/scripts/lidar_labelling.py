#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import numpy as np
from pypcd import pypcd
# import ros_numpy

pub = rospy.Publisher('/marker', Marker, queue_size=10)

def callback(msg):
    rospy.loginfo("I am receiving a new message with timestamp: %s.%s", msg.header.stamp.secs, msg.header.stamp.nsecs)

    # define datapath for txt and npy files
    datapath = "/home/felix/gokart-sensor/03_LiDAR_Labelling_npy/"

    # msg_ros_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    # mask = np.isfinite(msg_ros_np['x']) & np.isfinite(msg_ros_np['y']) & np.isfinite(msg_ros_np['z']) & np.isfinite(msg_ros_np['intensity'])
    # msg_ros_np = msg_ros_np[mask]
    # points = np.zeros(msg_ros_np.shape + (4,), dtype=np.float)
    # points[..., 0] = msg_ros_np['x']
    # points[..., 1] = msg_ros_np['y']
    # points[..., 2] = msg_ros_np['z']
    # points[..., 3] = msg_ros_np['intensity']
    # print(points)
    # points.reshape([-1, 4])
    # np.save(datapath + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".npy", points)
    # print(points)

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
    # bin name
    bin_path = datapath + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".bin"
    arr.astype('float32').tofile(bin_path)

    # input("Input something to continue the code\n")

    # marker = Marker()
    # marker.header.frame_id = msg.header.frame_id
    # # deletes all previous markers
    # marker.action = 3
    # pub.publish(marker)
    # # adds central marker at origin
    # marker = Marker()
    # marker.header.frame_id = msg.header.frame_id
    # marker.header.stamp = rospy.Time.now()
    # marker.action = 0
    # marker.type = 3
    # marker.id = -1
    # marker.color.a = 0.4
    # marker.color.r = 1
    # marker.color.g = 0
    # marker.color.b = 0
    # marker.scale.x = 1
    # marker.scale.y = 1
    # marker.scale.z = 3
    # marker.pose.orientation.x = 0
    # marker.pose.orientation.y = 0
    # marker.pose.orientation.z = 0
    # marker.pose.orientation.w = 1
    # marker.pose.position.x = 0
    # marker.pose.position.y = 0
    # marker.pose.position.z = 0
    # # marker.lifetime = rospy.Duration(0.3)
    # pub.publish(marker)
    #
    #
    # # open corresponding txt file to current LiDAR scan
    # f = open(datapath + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".txt", "w")
    # f.close()
    # f = open(datapath + str(msg.header.stamp.secs) + "_" + str(msg.header.stamp.nsecs) + ".txt", "r+")
    # # input number of cones as integer
    # while True:
    #     try:
    #         num_cones = int(input("How many cones does the current scene contain? Enter as int\n"))
    #         break
    #     except ValueError:
    #         print("Please enter an integer.")
    #
    # # for loop for every cone in the scan
    # for i in range(num_cones):
    #     print("Insert the top points x,y, and z coordinates")
    #     while True:
    #         try:
    #             top_point_x = float(input("Insert x position of top point:\n"))
    #             break
    #         except ValueError:
    #             print("Please enter a float.")
    #     top_point_y_str = input("Insert y position of top point:\n")
    #     top_point_y = float(top_point_y_str)
    #     top_point_z_str = input("Insert z position of top point:\n")
    #     top_point_z = float(top_point_z_str)
    #
    #     print("Insert the furthest left points x and y coordinates")
    #     left_point_x_str = input("Insert x position of left point:\n")
    #     left_point_x = float(left_point_x_str)
    #     left_point_y_str = input("Insert y position of left point:\n")
    #     left_point_y = float(left_point_y_str)
    #
    #     print("Insert the furthest right points x and y coordinates")
    #     right_point_x_str = input("Insert x position of right point:\n")
    #     right_point_x = float(right_point_x_str)
    #     right_point_y_str = input("Insert y position of right point:\n")
    #     right_point_y = float(right_point_y_str)
    #
    #     print("Insert the furthest front points x and y coordinates")
    #     front_point_x_str = input("Insert x position of front point:\n")
    #     front_point_x = float(front_point_x_str)
    #     front_point_y_str = input("Insert y position of front point:\n")
    #     front_point_y = float(front_point_y_str)
    #
    #     print("Insert the furthest rear points x and y coordinates")
    #     rear_point_x_str = input("Insert x position of rear point:\n")
    #     rear_point_x = float(rear_point_x_str)
    #     rear_point_y_str = input("Insert y position of rear point:\n")
    #     rear_point_y = float(rear_point_y_str)
    #
    #     print("Insert the lowest points z coordinate")
    #     lowest_point_z_str = input("Insert z position of lowest point:\n")
    #     lowest_point_z = float(lowest_point_z_str)
    #
    #     # angle to center point
    #     alpha = np.arctan(top_point_y / top_point_x)
    #     # incline of left and right border wall
    #     incline_lr = top_point_y / top_point_x
    #     # incline of front and rear border wall
    #     incline_fr = incline_lr + (np.pi / 2)
    #
    #     # using classic geometric properties dx and dy equal the shortest distance between two parallel lines
    #     b_left = left_point_y - (incline_lr * left_point_x)
    #     b_right = right_point_y - (incline_lr * right_point_x)
    #     b_front = front_point_y - (incline_fr * front_point_x)
    #     b_rear = rear_point_y - (incline_fr * rear_point_x)
    #     dx = np.abs(b_rear - b_front) / np.sqrt(1 + np.square(incline_fr))
    #     dy = np.abs(b_right - b_left) / np.sqrt(1 + np.square(incline_lr))
    #
    #     dz = top_point_z - lowest_point_z
    #
    #     x_center = ((b_right + 0.5 * (b_left - b_right)) - (b_front + 0.5 * (b_rear - b_front))) / (incline_fr - incline_lr)
    #     y_center = incline_lr * x_center + (b_right + 0.5 * (b_left - b_right))
    #     z_center = lowest_point_z + (0.5 * dz)
    #
    #     if i != 0:
    #         f.write("\n")
    #
    #     print(x_center)
    #     print(y_center)
    #     print(z_center)
    #     print(dx)
    #     print(dy)
    #     print(dz)
    #
    #     f.write(str(x_center) + " " + str(y_center) + " " + str(z_center) + " " + str(dx) + " " + str(dy) + " " + str(dz) + " " + str(alpha) + " Cone")
    #     f.truncate()
    #
    #     marker = Marker()
    #     marker.header.frame_id = msg.header.frame_id
    #     marker.header.stamp = rospy.Time.now()
    #     marker.action = 0
    #     marker.type = 1
    #     marker.id = i
    #     marker.color.a = 0.4
    #     marker.color.r = 0
    #     marker.color.g = 0
    #     marker.color.b = 1
    #     marker.scale.x = dx
    #     marker.scale.y = dy
    #     marker.scale.z = dz
    #     marker.pose.orientation.x = 0
    #     marker.pose.orientation.y = 0
    #     marker.pose.orientation.z = alpha
    #     marker.pose.orientation.w = 1
    #     marker.pose.position.x = x_center
    #     marker.pose.position.y = y_center
    #     marker.pose.position.z = z_center
    #     # marker.lifetime = rospy.Duration(0.3)
    #     pub.publish(marker)


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lidar_0/m1600/pcl2", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

