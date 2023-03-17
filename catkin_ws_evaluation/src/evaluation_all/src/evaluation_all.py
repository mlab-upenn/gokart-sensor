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
import os
import math



# TODO: define path to folder with txt files
path = "/home/felix/gokart-sensor/catkin_ws_evaluation/src/evaluation/src/support/full_dataset"
filelist = sorted(os.listdir(path))
print(filelist)

def callback(marker_array):

    global i
    global abs_num_true_positives
    global abs_num_false_positives
    global abs_num_false_negatives
    global abs_precision
    global abs_recall
    global abs_f1
    global abs_accuracy
    global end_eval_seen
    global abs_accuracy_positives
    global counter_accuracy_global

    timestamp_sec = filelist[i][:10]
    timestamp_nsec = filelist[i][11:-4]

    num_true_positives = 0
    num_false_negatives = 0

    if len(marker_array.markers) > 0:
        if marker_array.markers[0].header.stamp.secs > float(timestamp_sec):
            if marker_array.markers[0].header.stamp.nsecs > float(timestamp_nsec):
                if i < len(filelist) - 1:
                    i += 1
                    f_txt = open(path + "/" + filelist[i], "r+")

                    # # uncomment following 5 lines for normal MarkerArray
                    # marker_list = []
                    # for marker_single in marker_array.markers:
                    #     marker_list.append([marker_single.pose.position.x, marker_single.pose.position.y])
                    # marker_array = np.array(marker_list)
                    # print(marker_array)

                    # uncomment following 10 lines for OpenPCDet MarkerArray
                    marker_list = []
                    for marker_single in marker_array.markers:
                        # converts string back to a list, removes the blank spaces and appends it
                        temp_str = marker_single.text
                        temp_str = temp_str.replace("[", "")
                        temp_list = temp_str.split(" ")
                        temp_list = list(filter(None, temp_list))
                        marker_list.append([float(temp_list[0]), float(temp_list[1])])
                    marker_array = np.array(marker_list)


                    annotation_list = []
                    for line in f_txt:
                        temp_line = line.split(" ")
                        annotation_list.append([float(temp_line[0]) + math.cos(float(temp_line[6])) * 0.5 * float(temp_line[3]), float(temp_line[1])+ math.sin(float(temp_line[6])) * 0.5 * float(temp_line[3])])
                    annotation_array = np.array(annotation_list)

                    accuracy = 0
                    counter_accuracy = 0
                    accuracy_positives = 0

                    for j in range(annotation_array.shape[0]):
                        min_distance = 1000
                        found_positive = False
                        for k in range(marker_array.shape[0]):
                            dist = np.linalg.norm(annotation_array[j] - marker_array[k])
                            if dist < min_distance:
                                min_distance = dist
                            if dist < 0.33:
                                print("Annotation: " + str(annotation_array[j]) + " Marker: " + str(marker_array[k]) + " Distance: " + str(dist))
                                num_true_positives += 1
                                accuracy_positives += dist
                                found_positive = True
                                break

                        # determine accuracy
                        if min_distance < 5:
                            accuracy += min_distance
                            counter_accuracy += 1
                            counter_accuracy_global += 1

                        # determine number of false negatives
                        if not found_positive:
                            num_false_negatives += 1

                    # determine number of false positives
                    num_false_positives = marker_array.shape[0] - num_true_positives


                    if num_true_positives > 0:
                        accuracy = accuracy / counter_accuracy
                        accuracy_positives = accuracy_positives / num_true_positives



                    print("Timestamp: " + str(timestamp_sec) + "." + str(timestamp_nsec))
                    print("True Positives: " + str(num_true_positives))
                    print("False Positives: " + str(num_false_positives))
                    print("False Negatives: " + str(num_false_negatives))
                    print("Number of Annotations: " + str(annotation_array.shape[0]))
                    precision = num_true_positives / (num_true_positives + num_false_positives)
                    print("Precision: " + str(precision))
                    recall = num_true_positives / (num_true_positives + num_false_negatives)
                    print("Recall: " + str(recall))
                    f1 = 2 * num_true_positives / (2 * num_true_positives + num_false_positives + num_false_negatives)
                    print("F1-Score: " + str(f1))
                    print("Accuracy: " + str(accuracy))
                    print("Accuracy for True Positives: " + str(accuracy_positives))
                    print("")

                    abs_num_true_positives += num_true_positives
                    abs_num_false_positives += num_false_positives
                    abs_num_false_negatives += num_false_negatives
                    abs_precision += precision
                    abs_recall += recall
                    abs_f1 += f1
                    abs_accuracy += accuracy
                    abs_accuracy_positives += accuracy_positives

    if i == len(filelist) - 1 and end_eval_seen == False:
        print("Absolute Precision: " + str(abs_num_true_positives / (abs_num_true_positives + abs_num_false_positives)))
        print("Absolute Recall: " + str(abs_num_true_positives / (abs_num_true_positives + abs_num_false_negatives)))
        print("Absolute F1-Score: " + str(2 * abs_num_true_positives / (2 * abs_num_true_positives + abs_num_false_positives + abs_num_false_negatives)))
        print("Absolute Accuracy: " + str(abs_accuracy / counter_accuracy_global))
        print("Absolute Accuracy for True Positives: " + str(abs_accuracy_positives / abs_num_true_positives))
        print("Absolute True Positives: " + str(abs_num_true_positives))
        print("Absolute False Positives: " + str(abs_num_false_positives))
        print("Absolute False Negatives: " + str(abs_num_false_negatives))
        print("Average True Positives: " + str(abs_num_true_positives / len(filelist)))
        print("Average False Positives: " + str(abs_num_false_positives / len(filelist)))
        print("Average False Negatives: " + str(abs_num_false_negatives / len(filelist)))

        end_eval_seen = True



def evaluation():
    
    
    rospy.init_node('evaluation_all', anonymous=True)
    # TODO: change marker topic to /detect_3dbox for OpenPCDet and /cluster_markers for Geometric LiDAR Detection and /late_fusion_marker for Late Fusion
    global i
    i = 0
    global abs_num_true_positives
    global abs_num_false_positives
    global abs_num_false_negatives
    global abs_precision
    global abs_recall
    global abs_f1
    global abs_accuracy
    global abs_accuracy_positives
    global end_eval_seen
    global counter_accuracy_global
    abs_num_true_positives = 0
    abs_num_false_positives = 0
    abs_num_false_negatives = 0
    abs_precision = 0
    abs_recall = 0
    abs_f1 = 0
    abs_accuracy = 0
    end_eval_seen = False
    abs_accuracy_positives = 0
    counter_accuracy_global = 0

    rospy.Subscriber("/detect_3dbox", MarkerArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        evaluation()
    except rospy.ROSInterruptException:
        pass
