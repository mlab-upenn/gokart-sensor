#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import sys
import cv2
import os


class Yolov8(Node):

    def __init__(self):
        super().__init__('Yolov8')
        self.publisher_ = self.create_publisher(Image, 'topic_out', 10)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.Yolov8_ROS,
            10)
        self.model = YOLO("src/ros2_yolov8/ros2_yolov8/yolov8m.pt")
        os.chdir("src/ros2_yolov8/ros2_yolov8")
        self.subscription
        self.bridge = CvBridge()
        print(sys.version)

    def Yolov8_ROS(self, msg):

        # convert ROS image to cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # saves CV image at the predifined path
        filename = 'savedImage.jpg'
        cv2.imwrite(filename, cv_image)
        # Use the model with cv image
        self.model.predict(source='savedImage.jpg', return_outputs=True, show=True)
        # potential improvement in the future that outputs the image with bounding boxes back to ROS
        # print(type(predicted_image))
        # ros_image = self.bridge.cv2_to_imgmsg(predicted_image, "rgb8")
        # # ros_image = self.bridge.cv2_to_imgmsg(predicted_image.imgs[0])
        # self.publisher_.publish(ros_image)



def main(args=None):
    rclpy.init(args=args)
    obj = Yolov8()
    rclpy.spin(obj)
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
