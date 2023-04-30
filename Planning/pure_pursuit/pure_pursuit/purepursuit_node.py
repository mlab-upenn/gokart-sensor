import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, asin, sqrt, pi
import math
from collections import namedtuple
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped


"""
Constant Definition
"""
WIDTH = 0.2032  # (m)
WHEEL_LENGTH = 0.0381  # (m)
MAX_STEER = 0.36  # (rad)


class PurePursuit_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("purepursuit_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                # control
                ('minL', None),
                ('maxL', None),
                ('Lscale', None),
                ('minL_corner', None),
                ('maxL_corner', None),
                ('Lscale_corner', None),
                ('interpScale', None),

                ('minP', None),
                ('maxP', None),
                ('Pscale', None),
                ('minP_corner', None),
                ('maxP_corner', None),
                ('Pscale_corner', None),
                ('max_steer', None),
                ('D', None),
                ('vel_scale', None),
                # wp
                ('wp_path', None),
                ('wp_delim', None),
                ('wp_skiprows', None),
                # topics
                ('pose_topic', '/gnss_ekf'),
                ('drive_topic', '/automous_command_to_nucleo'),
                ('odom_topic', '/drive_info_from_nucleo')
            ])
        
        
        self.get_logger().info("python node initialized")
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('drive_topic').get_parameter_value().string_value, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').get_parameter_value().string_value, 
                                                  self.pose_cb,
                                                 10)
        self.odom_sub = self.create_subscription(AckermannDriveStamped, self.get_parameter('odom_topic').get_parameter_value().string_value, 
                                                 self.odom_cb,
                                                 10)
        
        self.lane = None
        self.last_lane = 0
        self.curr_vel = 0.0
        self.prev_ditem = 0.0
        self.prev_steer_error = 0.0

        # control parameters
        self.maxL = self.get_parameter('maxL').get_parameter_value().double_value
        self.minL = self.get_parameter('minL').get_parameter_value().double_value
        self.Lscale = self.get_parameter('Lscale').get_parameter_value().double_value
        self.maxL_corner = self.get_parameter('maxL_corner').get_parameter_value().double_value
        self.minL_corner = self.get_parameter('minL_corner').get_parameter_value().double_value
        self.Lscale_corner = self.get_parameter('Lscale_corner').get_parameter_value().double_value
        self.interpScale = self.get_parameter('interpScale').get_parameter_value().integer_value

        self.maxP = self.get_parameter('maxP').get_parameter_value().double_value
        self.minP = self.get_parameter('minP').get_parameter_value().double_value
        self.Pscale = self.get_parameter('Pscale').get_parameter_value().double_value
        self.maxP_corner = self.get_parameter('maxP_corner').get_parameter_value().double_value
        self.minP_corner = self.get_parameter('minP_corner').get_parameter_value().double_value
        self.Pscale_corner = self.get_parameter('Pscale_corner').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steer').get_parameter_value().double_value
        self.kd = self.get_parameter('D').get_parameter_value().double_value
        self.vel_scale = self.get_parameter('vel_scale').get_parameter_value().double_value

        self.load_wp(wp_path=self.get_parameter('wp_path').get_parameter_value().string_value,
                     delim=self.get_parameter('wp_delim').get_parameter_value().string_value,
                     skiprow=self.get_parameter('wp_skiprows').get_parameter_value().integer_value)

    def load_wp(self, wp_path:str, delim:str, skiprow:int):
        waypoints = np.loadtxt(wp_path, delimiter=delim, skiprows=skiprow)
        # 3 cols: x, y, v
        waypoints = np.vstack((waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])).T
        self.lane = np.expand_dims(waypoints, axis=0)
        print(self.lane)
        print(self.lane.shape)

    def pose_cb(self, pose_msg: PoseWithCovarianceStamped):
        
        cur_speed = self.curr_vel
        curr_x = pose_msg.pose.pose.position.x
        curr_y = pose_msg.pose.pose.position.y
        curr_pos = np.array([curr_x, curr_y])
        curr_quat = pose_msg.pose.pose.orientation
        curr_yaw = math.atan2(2 * (curr_quat.w * curr_quat.z + curr_quat.x * curr_quat.y),
                              1 - 2 * (curr_quat.y ** 2 + curr_quat.z ** 2))
        # self.get_logger().info(f'curr_yaw: {curr_yaw * 180/pi}')
        curr_pos_idx = np.argmin(np.linalg.norm(self.lane[0][:, :2] - curr_pos, axis=1))
        curr_lane_nearest_idx = np.argmin(np.linalg.norm(self.lane[self.last_lane][:, :2] - curr_pos, axis=1))
        traj_distances = np.linalg.norm(self.lane[self.last_lane][:, :2] - self.lane[self.last_lane][curr_lane_nearest_idx, :2], axis=1)
        segment_end = np.argmin(traj_distances)
        num_lane_pts = len(self.lane[self.last_lane])
        L = self.get_L_w_speed(cur_speed, False)
        # if curr_pos_idx in self.corner_wpIdx:
        #     L = self.get_L_w_speed(cur_speed, corner=True)
        # else:
        #     L = self.get_L_w_speed(cur_speed)
        while traj_distances[segment_end] <= L:
            segment_end = (segment_end + 1) % num_lane_pts
        segment_begin = (segment_end - 1 + num_lane_pts) % num_lane_pts
        x_array = np.linspace(self.lane[self.last_lane][segment_begin][0], self.lane[self.last_lane][segment_end][0], self.interpScale)
        y_array = np.linspace(self.lane[self.last_lane][segment_begin][1], self.lane[self.last_lane][segment_end][1], self.interpScale)
        v_array = np.linspace(self.lane[self.last_lane][segment_begin][2], self.lane[self.last_lane][segment_end][2], self.interpScale)
        xy_interp = np.vstack([x_array, y_array]).T
        dist_interp = np.linalg.norm(xy_interp-curr_pos, axis=1) - L
        i_interp = np.argmin(np.abs(dist_interp))
        target_global = np.array([x_array[i_interp], y_array[i_interp]])
        self.target_point = target_global
        target_v = v_array[i_interp]
        speed = target_v * self.vel_scale

        R = np.array([[np.cos(curr_yaw), np.sin(curr_yaw)],
                      [-np.sin(curr_yaw), np.cos(curr_yaw)]])
        target_local_x, target_local_y = R @ np.array([self.target_point[0] - curr_x,
                                           self.target_point[1] - curr_y])
        L = np.linalg.norm(curr_pos - target_global)
        gamma = 2 / L ** 2
        error = gamma * target_local_y
        steer = self.get_steer_w_speed(cur_speed, error)
        
        message = AckermannDriveStamped()
        message.drive.speed = speed
        message.drive.steering_angle = steer
        self.get_logger().info(f'speed: {speed}, steer: {steer}')
        self.drive_pub.publish(message)

    def odom_cb(self, odom: AckermannDriveStamped):
        self.curr_vel = odom.drive.speed

    def get_L_w_speed(self, speed, corner=False):
        if corner:
            interp_L_scale = (self.maxL_corner-self.minL_corner) / self.Lscale_corner
            return interp_L_scale * speed + self.minL_corner
        else:
            interp_L_scale = (self.maxL-self.minL) / self.Lscale
            return interp_L_scale * speed + self.minL

    def get_steer_w_speed(self, speed, error, corner=False):
        if corner:
            maxP = self.maxP_corner
            minP = self.minP_corner
            Pscale = self.Pscale_corner
        else:
            maxP = self.maxP
            minP = self.minP
            Pscale = self.Pscale

        interp_P_scale = (maxP-minP) / Pscale
        cur_P = maxP - speed * interp_P_scale

        d_error = error - self.prev_steer_error
        self.prev_ditem = d_error
        self.prev_steer_error = error
        if corner:
            steer = cur_P * error
        else:
            steer = cur_P * error + self.kd * d_error
        # print(f'cur_p_item:{cur_P * error},  cur_d_item:{kd * d_error}')
        new_steer = np.clip(steer, -self.max_steer, self.max_steer)
        return new_steer

        

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()