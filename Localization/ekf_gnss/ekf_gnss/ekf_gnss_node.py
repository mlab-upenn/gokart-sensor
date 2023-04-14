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

X = namedtuple('X', ['sec', 'nanosec', 'x', 'y', 'yaw', 'v'])
Stamped = namedtuple('Stamped', ['sec', 'nanosec', 'val'])


def get_Time(sec, nanosec):
    return sec + nanosec*1e-9

class Ekf_gnss_node(Node):
    def __init__(self):
        # here, super().__init__(<node_name>), while the node_name should be the same as provided in launch yaml file
        super().__init__("ekf_gnss_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 10),
            ])
        self.freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.dt = 1/self.freq
        
        self.ekf = EKF()
        
        # subscribe and publish
        self.get_logger().info("ekf_gnss node initialized, dt = %f" % self.dt)
        self.gnss_local_sub = self.create_subscription(PoseWithCovarianceStamped, "/gnss_local", self.gnss_cb, qos_profile=qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, qos_profile=qos_profile_sensor_data)
        self.ekf_pub = self.create_publisher(PoseWithCovarianceStamped, "/gnss_ekf", 10)
        self.timer = self.create_timer(self.dt, self.ekf_update)

        self.last_yaw = None
        self.curr_w = None

        self.last_x = None
        self.last_y = None 
        self.curr_v = None
        self.last_v = None
        self.X_est = None
        self.P_est = np.eye(4)
        self.initStatus_1 = False
        self.initStatus_2 = False

        self.msgLock = threading.Lock()
    

    def imu_cb(self, msg: Imu):
        # calculate the current yaw from orientation quaternion
        q = msg.orientation
        q = np.array([q.w, q.x, q.y, q.z])
        euler = transforms3d.euler.quat2euler(q)
        yaw = euler[2]
        # TODO: check why this equation not working
        # t3 = 2.0*(q.w*q.z + q.x*q.y)
        # t4 = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        # yaw = atan2(t3, t4) # -pi to pi

        # calculate the yaw rate
        if(not self.initStatus_1):
            self.last_yaw = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, yaw)
            return
        last_yaw = self.last_yaw.val
        if abs(yaw - last_yaw) > pi:
            if self.last_yaw.val > 0:
                last_yaw -= 2*pi
            else:
                last_yaw += 2*pi
        dt = get_Time(msg.header.stamp.sec, msg.header.stamp.nanosec) - get_Time(self.last_yaw.sec, self.last_yaw.nanosec)
        w = (yaw - last_yaw)/dt

        # self.get_logger().info(f"last_yaw = {last_yaw}, yaw = {yaw}, w = {w}, dt = {dt}")
        
        self.last_yaw = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, yaw)
        self.curr_w = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, w)


    def gnss_cb(self, msg:PoseWithCovarianceStamped):
        # calculate the current x and y position from the gnss data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # self.get_logger().info("x = %f, y = %f" % (x, y))
        # calculate the velocity
        if(not self.initStatus_1):
            self.last_x = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, x)
            self.last_y = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, y)
            return
        
        v = sqrt((x - self.last_x.val)**2 + (y - self.last_y.val)**2)/(get_Time(msg.header.stamp.sec, msg.header.stamp.nanosec) - get_Time(self.last_x.sec, self.last_x.nanosec))
        # self.get_logger().info("v = %f" % v)
        if(v>10.0):
            v = self.last_v
            msg.pose.covariance[8] = 0.5
        else:
            self.last_v = v
        
        self.last_x = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, x)
        self.last_y = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, y)
        self.curr_v = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, v)

        self.ekf.set_Q(msg.pose.covariance[8], msg.pose.covariance[8])
        # self.get_logger().info("gnss covariance = %f" % msg.pose.covariance[8])
        if(msg.pose.covariance[8] > 0.4):
            self.get_logger().info("gnss covariance is large")
            self.ekf.set_Q(5*msg.pose.covariance[8], 5*msg.pose.covariance[8])
    

    def checkStatus(self):
        if self.last_yaw is not None and self.last_x is not None and self.last_y is not None:
            self.initStatus_1 = True
        
        if self.curr_v is not None and self.curr_w is not None:
            self.initStatus_2 = True
            self.X_est = X(self.last_x.sec, self.last_x.nanosec, self.last_x.val, self.last_y.val, self.last_yaw.val, self.curr_v.val)
            self.get_logger().info("EKF initialized")
        
    
    def ekf_update(self):

        if(not self.initStatus_1 or not self.initStatus_2):
           self.checkStatus()
           return
        
        # get current time in sec and nanosec

        last_x_est_time = get_Time(self.X_est.sec, self.X_est.nanosec)
        curr_obs_time = get_Time(self.last_x.sec, self.last_x.nanosec)
        curr_w_time = get_Time(self.curr_w.sec, self.curr_w.nanosec)
        dt = curr_obs_time - last_x_est_time
        # self.get_logger().info(f"last_x_est_time:{last_x_est_time}, curr_obs_time:{curr_obs_time}, curr_w_time:{curr_w_time},  dt: {dt}")

        # self.get_logger().info(f"curr_v: {self.curr_v.val}, curr_w: {self.curr_w.val}")
        ud = np.array([[self.curr_v.val], [self.curr_w.val]])
        z = np.array([[self.last_x.val], [self.last_y.val]])
        # x_est = np.array([[self.X_est.x], [self.X_est.y], [self.X_est.yaw], [self.X_est.v]])
        x_est = np.array([[self.X_est.x], [self.X_est.y], [self.last_yaw.val], [self.X_est.v]])
        # self.get_logger().info(f"last x_est: {x_est[0][0]}, {x_est[1][0]}, {x_est[2][0]}, {x_est[3][0]}")
        # self.get_logger().info(f"yaw is {self.last_yaw.val}, v is {self.curr_v.val}")


        xEst, PEst = self.ekf.ekf_estimation(x_est, self.P_est, z, ud, dt, dead_reckoning=False)


        # if(self.ekf.Q_mea[-1][-1] > 0.4):
        #     xEst, PEst = self.ekf.ekf_estimation(x_est, self.P_est, z, ud, dt, dead_reckoning=False)
        #     self.get_logger().info("dead reckoning")
        #     self.get_logger().info(f"xEst: {xEst[0][0]}, {xEst[1][0]}, {xEst[2][0]}, {xEst[3][0]}")    
        # else:
        #     xEst, PEst = self.ekf.ekf_estimation(x_est, self.P_est, z, ud, dt, dead_reckoning=False)

        self.X_est = X(self.last_x.sec, self.last_x.nanosec, xEst[0][0], xEst[1][0], xEst[2][0], xEst[3][0])
        self.P_est = PEst

        # self.get_logger().info(f"X_est: {self.X_est.x}, {self.X_est.y}, {self.X_est.yaw}, {self.X_est.v}")
         
        # publish the estimated position
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = self.last_x.sec
        msg.header.stamp.nanosec = self.last_x.nanosec
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = xEst[0][0]
        msg.pose.pose.position.y = xEst[1][0]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(xEst[2][0]/2)
        msg.pose.pose.orientation.w = cos(xEst[2][0]/2)
        self.ekf_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Ekf_gnss_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()