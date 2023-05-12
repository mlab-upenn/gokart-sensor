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
        super().__init__("wp_record_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 2),
                ('pose_topic', '/gnss_ekf'),
                ('wp_path', "wp.csv"),
                ('velocity', 2.0)
            ])
        self.freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.dt = 1/self.freq
        self.v = self.get_parameter('velocity').get_parameter_value().double_value

        # subscribe and publish
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').get_parameter_value().string_value, 
                                                 self.pose_cb,
                                                 10)
        self.timer = self.create_timer(self.dt, self.record_wp)

        self.x = 0.0
        self.y = 0.0
        with open(self.get_parameter('wp_path').get_parameter_value().string_value, 'w') as f:
            f.write(str(self.x) + ',' + str(self.y) + ',' + str(self.v) + '\n')
                                               
    def pose_cb(self, pose_msg: PoseWithCovarianceStamped):
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y

    def record_wp(self):
        with open(self.get_parameter('wp_path').get_parameter_value().string_value, 'a') as f:
            f.write(str(self.x) + ',' + str(self.y) + ',' + str(self.v) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = Wp_record_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()