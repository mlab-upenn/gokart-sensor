#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

pub_processed_image = rospy.Publisher('yolo_image', Image, queue_size=10)
bridge = CvBridge()


def callback(msg):

    # Convert ROS Image to OpenCV Image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # flip image vertical
    cv_image = cv2.flip(cv_image, 1)
    # Convert OpenCV Image to ROS Image
    ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    # Publish ROS Image
    pub_processed_image.publish(ros_image)




def preprocessing():
    rospy.init_node('camera_preprocessing', anonymous=True)
    rospy.Subscriber("/rgb_publisher/color/image", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        preprocessing()
    except rospy.ROSInterruptException:
        pass
