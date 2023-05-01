import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, asin, sqrt, pi, radians
from collections import namedtuple
import numpy as np

Point = namedtuple("Point", ["lat", "lng", "x", "y"])
radius = 6371


class Python_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("gnss_to_local_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('projection_center_latitude', 39.9416),
                ('projection_center_longitude', -75.1993),
                ('projection_lat_span', 0.0010),
                ('projection_lon_span', 0.0016),
                ('debug_mode', False),
                ('set_origin', False),
                ('map_ori_path', None)
            ])

        # subscribe and publish
        self.gnss_sub = self.create_subscription(NavSatFix, "/navsatfix/use", self.gnss_cb, qos_profile=qos_profile_sensor_data)
        self.position_pub = self.create_publisher(PoseWithCovarianceStamped, "/gnss_local", 10)

        # init local frame
        # ^ y, North, lat
        # |
        # |
        #  ------>x, East, lon
        self.get_logger().info("gnss_to_local node initialized")
        # self.ori = Point(39.9411, -75.2001, 0, 0) # bottom left
        # self.diag = Point(39.9421, -75.1985, 0, 0) # top right

        #
        self.centerlat = self.get_parameter('projection_center_latitude').get_parameter_value().double_value
        self.centerlog = self.get_parameter('projection_center_longitude').get_parameter_value().double_value
        ori_lat = self.centerlat - self.get_parameter('projection_lat_span').get_parameter_value().double_value/2
        ori_lon = self.centerlog - self.get_parameter('projection_lon_span').get_parameter_value().double_value/2
        mid_lat_radian = radians(self.centerlat)
        self.ori = Point(ori_lat, ori_lon, 0, 0)
        #

        self.global_x_coff = radius*cos(mid_lat_radian)
        self.global_y_coff = radius
        self.init_ori()
        # self.get_logger().info(f"x_coff: {self.global_x_coff}, y_coff: {self.global_y_coff}, ori_x: {self.ori.x}, ori_y: {self.ori.y}")
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        if self.debug_mode:
            self.get_logger().info("debug mode on")
            self.get_logger().info(f"passed projection center latitude: {self.centerlat}")

        self.last_x = 0
        self.last_y = 0
        self.tolerance = 0.01
        self.set_origin = self.get_parameter('set_origin').get_parameter_value().bool_value

        if(not self.set_origin):
            self.get_logger().info("set origin to false, loading origin from file")
            with open(self.get_parameter('map_ori_path').get_parameter_value().string_value, 'r') as f:
                ori = np.loadtxt(f, delimiter=',', skiprows=0)
                self.ori = Point(ori[0], ori[1], 0, 0)
                self.init_ori()
    
    def init_ori(self):
        ori_x, ori_y = self.latlng2GlobalXY(self.ori.lat, self.ori.lng)
        self.ori = Point(self.ori.lat, self.ori.lng, ori_x, ori_y)
    
    def latlng2GlobalXY(self, lat, lng):
        x = self.global_x_coff*radians(lng)
        y = self.global_y_coff*radians(lat)
        return x, y

    def latlng2LocalXY(self, lat, lng):
        x, y = self.latlng2GlobalXY(lat, lng)
        x -= self.ori.x
        y -= self.ori.y
        # TODO: check the scale     
        return 1000*x, 1000*y
    
    def gnss_cb(self, msg:NavSatFix):
        if self.set_origin:
            self.ori = Point(msg.latitude, msg.longitude, 0, 0)
            with open(self.get_parameter('map_ori_path').get_parameter_value().string_value, 'w') as f:
                f.write(str(self.ori.lat) + ',' + str(self.ori.lng) + ',' + '\n')
            self.init_ori()
            self.set_origin = False #set origin once and then never set origin again

        x, y = self.latlng2LocalXY(msg.latitude, msg.longitude)
        self.last_x = x
        self.last_y = y
        pose = PoseWithCovarianceStamped()
        pose.header = msg.header
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = msg.position_covariance[0]
        pose.pose.covariance[4] = msg.position_covariance[4]
        self.position_pub.publish(pose)



def main(args=None):
    rclpy.init(args=args)
    node = Python_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()