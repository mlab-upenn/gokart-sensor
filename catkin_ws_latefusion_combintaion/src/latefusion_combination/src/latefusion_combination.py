import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import Image
import message_filters
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from cv_bridge import CvBridge
import cv2
from scipy.spatial import distance
import math
import time



rospy.init_node('latefusion_validation', anonymous=True)
pub_marker_array = rospy.Publisher("/late_fusion_marker", MarkerArray, queue_size=10)

# TODO: set value to 1 if openpcdet is used and 0 for everything else
use_openpc = 1


def callback(pnp_marker, det_marker):
    global average_time
    global average_time_counter

    # start time
    time_start = time.time()
    start_time = rospy.get_time()
    average_time_counter += 1
    print("---------------------------NEW CALLBACK---------------------------")

    marker_array = MarkerArray()

    # deletes all previous markers
    marker = Marker()
    marker.header.frame_id = "os_sensor"
    marker.id = 0
    marker.ns = "detections"
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)
    pub_marker_array.publish(marker_array)

    marker_array = MarkerArray()

    pnp_list = []
    for marker_single_pnp in pnp_marker.markers:
        pnp_list.append([marker_single_pnp.pose.position.x, marker_single_pnp.pose.position.y])
    pnp_array = np.array(pnp_list)
    print(pnp_array)

    det_list = []
    for marker_single_openpc in det_marker.markers:
        # converts string back to a list, removes the blank spaces and appends it
        temp_str = marker_single_openpc.text
        temp_str = temp_str.replace("[","")
        temp_str = temp_str.replace("]", "")
        temp_list = temp_str.split(" ")
        temp_list = list(filter(None, temp_list))
        print(temp_list)
        det_list.append([float(temp_list[0]), float(temp_list[1]), float(temp_list[2]), float(temp_list[3]), float(temp_list[4]), float(temp_list[6])])

    det_array = np.array(det_list)
    print(det_array)

    # Potential improvement when more trust in PnP
    # if pnp_array.shape[0] <= det_array.shape[0]:
    #     shorter_array = pnp_array
    #     longer_array = det_array
    # else:
    #     shorter_array = det_array
    #     longer_array = pnp_array

    for i in range(det_array.shape[0]):
        print(det_array[i][:2])
        print(det_array[i])
        # centerpoint OpenPCDet correction
        det_array[i][0] = det_array[i][0] + math.cos(det_array[i][5]) * det_array[i][3] / 2
        det_array[i][1] = det_array[i][1] + math.sin(det_array[i][5]) * det_array[i][3] / 2
        closest_point_indx = distance.cdist(det_array[i][:2].reshape(1, -1),pnp_array).argmin()
        closest_point_pnp = pnp_array[closest_point_indx]

        d = math.sqrt(pow((det_array[i][0]-closest_point_pnp[0]),2) + pow((det_array[i][1]-closest_point_pnp[1]),2))
        if d < 1.0:

            # #combine x and y coordinates
            pos_x = det_array[i][0] + 0.5 * (closest_point_pnp[0] - det_array[i][0])
            pos_y = det_array[i][1] + 0.5 * (closest_point_pnp[1] - det_array[i][1])

            # # x and y coordinates solely from OpenPCDet
            pos_x = det_array[i][0]
            pos_y = det_array[i][1]


            # time correction for late fusion
            end_time = rospy.get_time()

            marker = Marker()
            marker.header.frame_id = "os_sensor"
            marker.header.stamp = rospy.Time.now() - rospy.Duration(end_time - start_time) #.1 besster Wert (30 False Positives)
            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.1)
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.32
            marker.scale.y = 0.32
            marker.scale.z = 0.7
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.pose.position.x = pos_x #det_array[i][0]
            marker.pose.position.y = pos_y #det_array[i][1]
            marker.pose.position.z = det_array[i][2]
            marker.color.a = 0.4
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)


    # end time and print time elapsed
    print("Time elapsed: ", time.time() - time_start, " seconds")
    average_time += time.time() - time_start
    print("Average time elapsed: ", average_time / average_time_counter, " seconds")
    pub_marker_array.publish(marker_array)






def late_fusion_validation():
    
    global average_time
    global average_time_counter
    average_time = 0
    average_time_counter = 0

    # approximate time synchroniser for MarkerArray and BoundingBoxes
    # TODO: change det_marker_sub topic to /detect_3dbox for OpenPCDet and /cluster_markers for Geometric LiDAR Detection
    pnp_marker_sub = message_filters.Subscriber('/marker', MarkerArray)
    det_marker_sub = message_filters.Subscriber('/detect_3dbox', MarkerArray)
    ts = message_filters.ApproximateTimeSynchronizer([pnp_marker_sub, det_marker_sub], 5, 0.05, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        late_fusion_validation()
    except rospy.ROSInterruptException:
        pass
