import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sin, cos, asin, sqrt, pi, radians, atan2
from collections import namedtuple
from ekf_gnss.ekf import EKF
import numpy as np
import threading
import transforms3d


class Wp_record_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("wp_record_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 10),
                ('pose_topic', '/gnss_ekf'),
            ])
        self.freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.dt = 1/self.freq
        # subscribe and publish
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').get_parameter_value().string_value, 
                                                 self.pose_cb,
                                                 10)
        self.timer = self.create_timer(self.dt, self.record_wp)

        self.x = 0.0
        self.y = 0.0
                                               
    def pose_cb(self, pose_msg: PoseWithCovarianceStamped):
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y

    def record_wp(self):
        return       
