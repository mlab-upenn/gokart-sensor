import rclpy
from rclpy.node import Node
from math import pi, atan2, cos, sin
from queue import PriorityQueue

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from queue import PriorityQueue
from rclpy.qos import qos_profile_sensor_data


class Gap_follow_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in yaml file
        super().__init__("gap_follow_node")

        # load yolov8 model
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', None),
                ('drive_topic', '/automous_command_to_nucleo'),
                ('danger_thres', 1.0),
                ('safe_radius', 20),
                ('safe_thres', 2.0),
                ('max_gap', 250),
                ('min_gap', 30),
                ('min_boundary', 10),
                ('max_boundary', 170),
                ('max_speed', 3.0),
                ('min_speed', 2.0),
                ('steer_speed_scale', 0.87)
            ])
    
        self.scan_subscriber = self.create_subscription(LaserScan, self.get_parameter("scan_topic").get_parameter_value().string_value, self.scan_cb, qos_profile=qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').get_parameter_value().string_value, 10)
        self.danger_thres = self.get_parameter('danger_thres').get_parameter_value().double_value
        self.safe_radius = self.get_parameter('safe_radius').get_parameter_value().integer_value
        self.safe_thres = self.get_parameter('safe_thres').get_parameter_value().double_value
        self.max_gap = self.get_parameter('max_gap').get_parameter_value().integer_value
        self.min_gap = self.get_parameter('min_gap').get_parameter_value().integer_value
        self.min_boundary = self.get_parameter('min_boundary').get_parameter_value().integer_value
        self.max_boundary = self.get_parameter('max_boundary').get_parameter_value().integer_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.steer_speed_scale = self.get_parameter('steer_speed_scale').get_parameter_value().double_value

    
    def find_best_point(self, start_idx, end_idx, ranges):
        safe_range = PriorityQueue()
        safe_p_left = start_idx
        safe_p_right = end_idx
        p = start_idx
        while p < end_idx:
            if ranges[p] >= self.safe_thres:
                safe_p_left = p
                p += 1
                while p < end_idx and ranges[p] >= self.safe_thres and p-safe_p_left <= self.max_gap:
                    p += 1
                safe_p_right = p-1
                if safe_p_right != safe_p_left:
                    safe_range.put((-(np.max(ranges[safe_p_left:safe_p_right])), (safe_p_left, safe_p_right)))
            else:
                p += 1
        target = np.argmax(ranges)
        if safe_range.empty():
            self.get_logger().info('no safe range, return farmost point')
            return target
        else:
            while not safe_range.empty():
                safe_p_left, safe_p_right = safe_range.get()[1]
                target = (safe_p_left+safe_p_right)//2
                if self.min_boundary <= target <= self.max_boundary and abs(safe_p_right-safe_p_left) >= self.min_gap:
                    return target
            self.get_logger().info('best point not in boundary, return farmost point')
            return target


    def scan_cb(self, scan_msg):
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min        
        ranges = scan_msg.ranges
        n = len(ranges)
        proc_ranges = np.array(ranges)
        # self.get_logger().info(f"len(ranges):{n}")
        target_idx = self.find_best_point(0, n, proc_ranges)
        steer = angle_min + target_idx * angle_increment
        steer = np.clip(steer, -self.steer_speed_scale, self.steer_speed_scale)
        speed = self.max_speed - (self.max_speed - self.min_speed) * abs(steer)/ self.steer_speed_scale
        # self.get_logger().info('target_idx:{}, steer:{}, speed:{}'.format(target_idx, steer, speed))
        ackerman_msg = AckermannDriveStamped()
        ackerman_msg.header.stamp = self.get_clock().now().to_msg()
        ackerman_msg.drive.steering_angle = steer
        ackerman_msg.drive.speed = speed
    

def main(args=None):
    rclpy.init(args=args)
    node = Gap_follow_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()