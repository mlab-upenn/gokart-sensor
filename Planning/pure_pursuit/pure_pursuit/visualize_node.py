#!/usr/bin/env python3
import numpy as np
import math
import os
from typing import Union

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Int16
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped


"""
Constant Definition
"""
WIDTH = 0.2032  # (m)
WHEEL_LENGTH = 0.0381  # (m)
MAX_STEER = 0.36  # (rad)


class LaneVisualize(Node):
    """
    Class for lane visualization
    """

    def __init__(self):
        super().__init__("visualize_node")

        # ROS Params
        self.declare_parameters(
            namespace='',
            parameters=[
                # wp
                ('wp_path', None),
                ('wp_delim', None),
                ('wp_skiprows', None),
                ('config_path', None),
                ('overtake_wp_name', None),
                ('corner_wp_name', None),
            ])

        # load wp
        self.load_wp(wp_path=self.get_parameter('wp_path').get_parameter_value().string_value,
                     delim=self.get_parameter('wp_delim').get_parameter_value().string_value,
                     skiprow=self.get_parameter('wp_skiprows').get_parameter_value().integer_value)
        self.corner_idx = np.load(self.get_parameter('config_path').get_parameter_value().string_value + '/' + self.get_parameter('corner_wp_name').get_parameter_value().string_value)
        self.corner_idx = set(self.corner_idx)
        # Topics & Subs, Pubs
        self.timer = self.create_timer(1.0, self.timer_callback)
        traj_topic = "/global_path/optimal_trajectory"
        self.traj_pub_ = self.create_publisher(Marker, traj_topic, 10)
        # self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').get_parameter_value().string_value, 
        #                                           self.pose_cb,
        #                                          10)
        # self.odom_sub = self.create_subscription(AckermannDriveStamped, self.get_parameter('odom_topic').get_parameter_value().string_value, 
        #                                          self.odom_cb,
        #                                          10)

    def load_wp(self, wp_path:str, delim:str, skiprow:int):
        waypoints = np.loadtxt(wp_path, delimiter=delim, skiprows=skiprow)
        # 3 cols: x, y, v
        waypoints = np.vstack((waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])).T
        self.lane = np.expand_dims(waypoints, axis=0)
        self.traj_x = waypoints[:, 0]
        self.traj_y = waypoints[:, 1]
        self.traj_v = waypoints[:, 2]
        self.num_traj_pts = len(self.traj_x)
        self.v_max = 6.0
        self.v_min = 0.0

    def timer_callback(self):
        self.visualize_global_path()

    def visualize_global_path(self):
        # Publish trajectory waypoints
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 0
        marker.ns = "global_planner"
        marker.type = 4
        marker.action = 0
        marker.points = []
        marker.colors = []

        for i in range(self.num_traj_pts + 1):
            this_point = Point()
            this_point.x = self.traj_x[i % self.num_traj_pts]
            this_point.y = self.traj_y[i % self.num_traj_pts]
            marker.points.append(this_point)

            this_color = ColorRGBA()
            if(i in self.corner_idx):
                this_color.r = 0.0
                this_color.g = 0.0
                this_color.b = 1.0
            else:
                speed_ratio = (self.traj_v[i % self.num_traj_pts] - self.v_min) / (self.v_max - self.v_min)
                this_color.a = 1.0
                this_color.r = (1 - speed_ratio)
            this_color.g = speed_ratio
            marker.colors.append(this_color)

        this_scale = 0.1
        marker.scale.x = this_scale
        marker.scale.y = this_scale
        marker.scale.z = this_scale

        marker.pose.orientation.w = 1.0

        self.traj_pub_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    print("Lane Visualize Initialized")
    lane_visualize_node = LaneVisualize()
    rclpy.spin(lane_visualize_node)

    lane_visualize_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
