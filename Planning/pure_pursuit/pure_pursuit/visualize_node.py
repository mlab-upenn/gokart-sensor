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
import trajectory_planning_helpers as tph


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
                ('wp_track_width', None),
                ('wp_centerline_filename', None),
                ('wp_filename', None),
                ('wp_delim', None),
                ('wp_skiprows', None),
                ('config_path', None),
                ('wp_overtake_filename', None),
                ('wp_corner_filename', None),
                ('wp_x_idx', None),
                ('wp_y_idx', None),
                ('wp_v_idx', None),
            ])

        # load wp
        self.load_wp(wp_path=os.path.join(self.get_parameter('config_path').value, self.get_parameter('wp_filename').value),
                     delim=self.get_parameter('wp_delim').get_parameter_value().string_value,
                     skiprow=self.get_parameter('wp_skiprows').get_parameter_value().integer_value)
        self.corner_idx = np.load(self.get_parameter('config_path').get_parameter_value().string_value + '/' + self.get_parameter('wp_corner_filename').get_parameter_value().string_value)
        self.corner_idx = set(self.corner_idx)
        # Topics & Subs, Pubs
        self.timer = self.create_timer(1.0, self.timer_callback)
        traj_topic = "/global_path/optimal_trajectory"
        self.traj_pub_ = self.create_publisher(Marker, traj_topic, 10)
        self.bound1_pub_ = self.create_publisher(Marker, "/bound1", 10)
        self.bound2_pub_ = self.create_publisher(Marker, "/bound2", 10)
        self.purepusuit_sub_ = self.create_subscription(Point, "/pp_target", self.purepursuit_cb, 10)
        self.purepursuit_target_pub_ = self.create_publisher(Marker, "/pp_target_marker", 10)

    def load_wp(self, wp_path:str, delim:str, skiprow:int):
        waypoints = np.loadtxt(wp_path, delimiter=delim, skiprows=skiprow)
        # 3 cols: x, y, v
        waypoints = np.vstack((waypoints[:, self.get_parameter("wp_x_idx").get_parameter_value().integer_value], 
                               waypoints[:, self.get_parameter("wp_y_idx").get_parameter_value().integer_value], 
                               waypoints[:, self.get_parameter("wp_v_idx").get_parameter_value().integer_value])).T
        centerline = np.loadtxt(os.path.join(self.get_parameter('config_path').value, self.get_parameter('wp_centerline_filename').value), delimiter=',', skiprows=0)
        centerline = np.vstack((centerline[:, 0], centerline[:, 1])).T
        centerline_close = np.vstack((centerline, centerline[0, :]))
        _, _, _, normvec = tph.calc_splines.calc_splines(path=centerline_close)
        track_span = self.get_parameter('wp_track_width').get_parameter_value().double_value/2
        track_span_arr = np.ones((centerline.shape[0], 1)) * track_span
        self.bound1 = centerline + normvec * track_span_arr
        self.bound2 = centerline - normvec * track_span_arr

        self.lane = np.expand_dims(waypoints, axis=0)
        self.traj_x = waypoints[:, 0]
        self.traj_y = waypoints[:, 1]
        self.traj_v = waypoints[:, 2]
        self.num_traj_pts = len(self.traj_x)
        self.v_max = np.max(self.traj_v)
        self.v_min = np.min(self.traj_v)

    def purepursuit_cb(self, msg:Point):
        # Publish target waypoint
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 0
        marker.ns = "target_waypoint"
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        this_scale = 0.2
        marker.scale.x = this_scale
        marker.scale.y = this_scale
        marker.scale.z = this_scale
        marker.pose.orientation.w = 1.0
        marker.lifetime.nanosec = int(1e8)
        self.purepursuit_target_pub_.publish(marker)


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
            # if(i in self.corner_idx):
            if False:
                this_color.r = 0.0
                this_color.g = 0.0
                this_color.b = 1.0
            else:
                speed_ratio = (self.traj_v[i % self.num_traj_pts] - self.v_min) / (self.v_max - self.v_min)
                this_color.a = 1.0
                this_color.r = speed_ratio
                this_color.g = 1-speed_ratio
            marker.colors.append(this_color)

        this_scale = 0.1
        marker.scale.x = this_scale
        marker.scale.y = this_scale
        marker.scale.z = this_scale

        marker.pose.orientation.w = 1.0

        self.traj_pub_.publish(marker)
        
        # Publish bound1
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 0
        marker.ns = "global_planner"
        marker.type = 4
        marker.action = 0
        marker.points = []
        marker.colors = []
        for i in range(self.bound1.shape[0] + 1):
            this_point = Point()
            this_point.x = self.bound1[i % self.bound1.shape[0], 0]
            this_point.y = self.bound1[i % self.bound1.shape[0], 1]
            marker.points.append(this_point)
            this_color = ColorRGBA()
            this_color.a = 1.0
            this_color.r = 1.0
            this_color.g = 1.0
            this_color.b = 1.0
            marker.colors.append(this_color)
            this_scale = 0.1
            marker.scale.x = this_scale
            marker.scale.y = this_scale
            marker.scale.z = this_scale
            marker.pose.orientation.w = 1.0
        self.bound1_pub_.publish(marker)

        # Publish bound2
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 0
        marker.ns = "global_planner"
        marker.type = 4
        marker.action = 0
        marker.points = []
        marker.colors = []
        for i in range(self.bound2.shape[0] + 1):
            this_point = Point()
            this_point.x = self.bound2[i % self.bound2.shape[0], 0]
            this_point.y = self.bound2[i % self.bound2.shape[0], 1]
            marker.points.append(this_point)
            this_color = ColorRGBA()
            this_color.a = 1.0
            this_color.r = 1.0
            this_color.g = 1.0
            this_color.b = 1.0
            marker.colors.append(this_color)
            this_scale = 0.1
            marker.scale.x = this_scale
            marker.scale.y = this_scale
            marker.scale.z = this_scale
            marker.pose.orientation.w = 1.0
        self.bound2_pub_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    print("Lane Visualize Initialized")
    lane_visualize_node = LaneVisualize()
    rclpy.spin(lane_visualize_node)

    lane_visualize_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
