import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, asin, sqrt, pi
from collections import namedtuple

Point = namedtuple("Point", ["lat", "lng", "x", "y"])
radius = 6371


class Python_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("python_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', None)
            ])

        # subscribe and publish
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        # self._sub = self.create_subscription(
        #     Image, self.image_topic, self.image_cb,
        #     qos_profile_sensor_data
        # )
        self.get_logger().info("python node initialized")
        self.p0 = Point(39.942, -75.199, 0, 0) # top left
        self.p1 = Point(39.941, -75.198, 0, 0) # bottom right

        self.last_x = 0
        self.last_y = 0
        self.tolerance = 0.01

    def latlng2GlobalXY(self, lat, lng):
        x = radius*lng*cos((self.p0.lat + self.p1.lat)/2)
        y = radius*lat
        return x, y

    def latlng2LocalXY(self, lat, lng):
        x, y = self.latlng2GlobalXY(lat, lng)
        x -= self.p0.x
        y -= self.p0.y
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = Python_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()