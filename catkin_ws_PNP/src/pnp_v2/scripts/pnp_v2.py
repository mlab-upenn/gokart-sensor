#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
import imutils
import time


# define publishers
pub_marker = rospy.Publisher('/marker', MarkerArray, queue_size=10)
pub_cropped_image = rospy.Publisher('/Image_cropped_bb', Image, queue_size=10)
pub_bb_image = rospy.Publisher('/Image_bb', Image, queue_size=10)

# TODO: cone parameter and LiDAR height over ground
cone_height_meter = 0.7
cone_diameter_meter = 0.32
lidar_height_over_ground = 1.26

# correction of marker to ground plane
marker_correction = - (lidar_height_over_ground - (cone_height_meter / 2))

# TODO: input calibration matrix from LiDAR to camera frame
l_t_c = np.array([[0.0190983, -0.999815, 0.00232702, 0.0440449],
                  [-0.193816, -0.0059855, -0.98102, 0.142687],
                  [0.980852, 0.0182848, -0.193894, -0.0378563],
                  [0, 0, 0, 1]])

# TODO: input intrinsic camera parameters from camera calibration matrix
fx = 1442.193090
fy = 1440.905704
cx = 943.936435
cy = 542.578706

def callback(image_message, bb_message):
    global average_time
    global average_time_counter
    time_start = time.time()
    average_time_counter += 1
    bb_arr = bb_message.bounding_boxes
    i = 0

    marker_array = MarkerArray()

    # deletes all previous markers
    marker = Marker()
    marker.header.frame_id = "os_sensor"
    marker.id = 0
    marker.ns = "detections"
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)
    pub_marker.publish(marker_array)

    marker_array = MarkerArray()

    # convert image from ROS image to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

    for bs in bb_arr:


        # crop image to original bounding box
        x_min_orig = int(bs.xmin)
        x_max_orig = int(bs.xmax)
        y_min_orig = int(bs.ymin)
        y_max_orig = int(bs.ymax)
        bb_img_bgr = cv_image[y_min_orig:y_max_orig, x_min_orig:x_max_orig]

        # convert image back to ROS image and publish
        bb_image_message = bridge.cv2_to_imgmsg(bb_img_bgr, encoding="passthrough")
        pub_bb_image.publish(bb_image_message)

        # crop image to bounding box +- 10%
        x_min = int(bs.xmin * 0.9)
        x_max = int(bs.xmax * 1.1)
        y_min = int(bs.ymin * 0.9)
        y_max = int(bs.ymax * 1.1)
        crop_img_bgr = cv_image[y_min:y_max, x_min:x_max]

        # convert cv image to hsv colorspace
        crop_img_hsv = cv2.cvtColor(crop_img_bgr, cv2.COLOR_BGR2HSV)

        # initial in range approach only regarding hue
        # h, s, v = cv2.split(crop_img_hsv)
        # h_threshold = cv2.inRange(h, 0, 60)



        ORANGE_MIN = np.array([0, 50, 50], np.uint8)
        ORANGE_MAX = np.array([25, 255, 255], np.uint8)
        frame_threshed = cv2.inRange(crop_img_hsv, ORANGE_MIN, ORANGE_MAX)

        ret_h, thresh_h = cv2.threshold(frame_threshed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cnts_h = cv2.findContours(thresh_h, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts_h = imutils.grab_contours(cnts_h)
        try:
            c_h = max(cnts_h, key=cv2.contourArea)
            x_h, y_h, w_h, h_h = cv2.boundingRect(c_h)
            bb_state = 1
        except ValueError:
            print("No contour found, bounding box skipped")
            bb_state = 0

        # if there is no orange in the image skip the rest of the loop
        if bb_state == 1:
            s_threshold = crop_img_hsv[:, :, 1]
            ret_s, thresh_s = cv2.threshold(s_threshold, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            cnts_s = cv2.findContours(thresh_s, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts_s = imutils.grab_contours(cnts_s)
            try:
                c_s = max(cnts_s, key=cv2.contourArea)
                x_s, y_s, w_s, h_s = cv2.boundingRect(c_s)

                # intersection of both rectangles
                x = max(x_h, x_s)
                y = max(y_h, y_s)
                w = min(x_h + w_h, x_s + w_s) - x
                h = min(y_h + h_h, y_s + h_s) - y
            except ValueError:
                x = x_h
                y = y_h
                w = w_h
                h = h_h

            cv2.rectangle(crop_img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # convert image back to ROS image and publish
            cropped_image_message = bridge.cv2_to_imgmsg(crop_img_bgr, encoding="passthrough")
            pub_cropped_image.publish(cropped_image_message)

            # calculate position of center and height of bounding box
            x_center_pixel = (bs.xmax - bs.xmin) * 0.5 + bs.xmin
            y_center_pixel = (bs.ymax - bs.ymin) * 0.5 + bs.ymin
            height_pixel = (bs.ymax - bs.ymin)

            # calculate x, y, and z position in camera frame
            Z_meter_cam = (fy * cone_height_meter) / height_pixel
            X_meter_cam = ((x_center_pixel - cx) * Z_meter_cam) / fx
            Y_meter_cam = ((y_center_pixel - cy) * Z_meter_cam) / fy

            # convert cartesian coordinates in homogenous coordinates and transform to LiDAR frame
            point_meter_cam = np.array([X_meter_cam, Y_meter_cam, Z_meter_cam, 1])
            point_meter_lidar = np.linalg.inv(l_t_c) @ point_meter_cam

            # publish detections as markers for RViZ
            marker = Marker()
            marker.header.frame_id = "os_sensor"
            marker.header.stamp = bb_message.header.stamp
            marker.ns = "detections"
            marker.id = i
            marker.type = 3
            marker.action = 0
            marker.scale.x = cone_diameter_meter
            marker.scale.y = cone_diameter_meter
            marker.scale.z = cone_height_meter
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.pose.position.x = point_meter_lidar[0]
            marker.pose.position.y = point_meter_lidar[1]
            # if z position required, uncomment next line
            # marker.pose.position.z = point_meter_lidar[2]
            marker.pose.position.z = marker_correction
            marker.color.a = 0.4
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            i += 1
            marker_array.markers.append(marker)

    # to print time per iteration, uncomment next line
    print("Time elapsed: ", time.time() - time_start, " seconds")
    average_time += time.time() - time_start
    print("Average time elapsed: ", average_time / average_time_counter, " seconds")

    pub_marker.publish(marker_array)





def listener():
    global average_time
    global average_time_counter
    average_time = 0
    average_time_counter = 0

    rospy.init_node('pnp_v2', anonymous=True)
    # TODO: change topic name to correct camera topic
    image_sub = message_filters.Subscriber('/rgb_publisher/color/image', Image)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, bb_sub], 5, 1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

