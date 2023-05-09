import rclpy
from rclpy.node import Node
from math import pi, atan2, cos, sin
from queue import PriorityQueue
import copy

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from queue import PriorityQueue
from rclpy.qos import qos_profile_sensor_data


@staticmethod
def sliding_window_maximum(sequence, window_size):
    n = len(sequence)
    padding = window_size // 2
    padded_sequence = np.pad(sequence, (padding, padding), mode='edge')
    result = np.empty(n)

    # Create a 2D array with sliding windows
    windows = np.lib.stride_tricks.sliding_window_view(padded_sequence, window_shape=window_size)

    # Find the minimum value within each window
    result = np.max(windows, axis=1)

    return result

@staticmethod
def sliding_window_minimum(sequence, window_size):
    n = len(sequence)
    padding = window_size // 2
    padded_sequence = np.pad(sequence, (padding, padding), mode='edge')
    result = np.empty(n)

    # Create a 2D array with sliding windows
    windows = np.lib.stride_tricks.sliding_window_view(padded_sequence, window_shape=window_size)

    # Find the minimum value within each window
    result = np.min(windows, axis=1)

    return result



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
                ('safe_thres', 2.0),
                ('window_size', 8),
                ('max_gap', 250),
                ('min_gap', 30),
                ('min_boundary', 10),
                ('max_boundary', 170),
                ('max_speed', 3.0),
                ('min_speed', 2.0),
                ('steer_speed_scale', 0.87)
            ])
    
        self.scan_subscriber = self.create_subscription(LaserScan, self.get_parameter("scan_topic").get_parameter_value().string_value, self.scan_cb, qos_profile=qos_profile_sensor_data)
        self.scan_pub_debug = self.create_publisher(LaserScan, '/scan_debug', qos_profile_sensor_data)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').get_parameter_value().string_value, 10)
        self.safe_thres = self.get_parameter('safe_thres').get_parameter_value().double_value
        self.max_gap = self.get_parameter('max_gap').get_parameter_value().integer_value
        self.min_gap = self.get_parameter('min_gap').get_parameter_value().integer_value
        self.min_boundary = self.get_parameter('min_boundary').get_parameter_value().integer_value
        self.max_boundary = self.get_parameter('max_boundary').get_parameter_value().integer_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.steer_speed_scale = self.get_parameter('steer_speed_scale').get_parameter_value().double_value
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

    def pre_process(self, ranges):
        # remove noise in a gap, but erode the noise to become a gap inside an obstacle(can be later filted by setting min_gap)
        proc_ranges = sliding_window_maximum(proc_ranges, self.window_size)
        return proc_ranges
        

    def find_best_point(self, start_idx, end_idx, ranges):
        safe_range = PriorityQueue()
        safe_range_debug = PriorityQueue()
        safe_p_left = start_idx
        safe_p_right = end_idx
        p = start_idx
        while p < end_idx:
            if ranges[p] >= self.safe_thres:
                safe_p_left = p
                p += 1
                while p < end_idx and ranges[p] >= self.safe_thres:
                    p += 1
                # while ranges[p] >= self.safe_thres or (max(ranges[p-2:p+1] >= self.safe_thres and ranges[p+1] >= self.safe_radius):
                    # p += 1
                safe_p_right = p-1
                if abs(safe_p_right-safe_p_left) >= self.min_gap:
                    safe_range.put((-(safe_p_right-safe_p_left), (safe_p_left, safe_p_right)))
                    safe_range_debug.put((-(safe_p_right-safe_p_left), (safe_p_left, safe_p_right)))
            else:
                p += 1
        target = np.argmax(ranges)
        if safe_range.empty():
            # self.get_logger().info('no safe range, return farmost point')
            return target, safe_range_debug
        else:
            while not safe_range.empty():
                safe_p_left, safe_p_right = safe_range.get()[1]
                target = (safe_p_left+safe_p_right)//2
                if  abs(safe_p_right-safe_p_left) >= self.min_gap:
                    return target, safe_range_debug
            # self.get_logger().info('best point not in boundary, return farmost point')
            return target, safe_range_debug


    def scan_cb(self, scan_msg):
        angle_increment = scan_msg.angle_increment
        angle_min = scan_msg.angle_min        
        ranges = scan_msg.ranges
        n = len(ranges)
        proc_ranges = np.array(ranges)
        # self.get_logger().info(f"len(ranges):{n}")
        target_idx, safe_range_debug = self.find_best_point(self.min_boundary, self.max_boundary, proc_ranges)
        # debug
        scan_debug = copy.deepcopy(scan_msg)
        debug_ranges = [0.0] * n
        while not safe_range_debug.empty():
            safe_p_left, safe_p_right = safe_range_debug.get()[1]
            # self.get_logger().info('safe range: ({}, {})'.format(safe_p_left, safe_p_right))
            debug_ranges[safe_p_left:safe_p_right] = [self.safe_thres] * (safe_p_right-safe_p_left)
        debug_ranges[target_idx] = self.safe_thres * 0.8
        scan_debug.ranges = debug_ranges
        self.scan_pub_debug.publish(scan_debug)
        # debug
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


"""

"""