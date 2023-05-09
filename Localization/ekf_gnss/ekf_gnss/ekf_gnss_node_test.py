import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
from ackermann_msgs.msg import AckermannDriveStamped
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sin, cos, asin, sqrt, pi, radians, atan2
from collections import namedtuple
from ekf_gnss.ekf import EKF
import numpy as np
import threading
import transforms3d

X = namedtuple('X', ['sec', 'nanosec', 'x', 'y', 'yaw', 'v','omega'])
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
                ('frequency', 20),
            ])
        self.freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.dt = 1/self.freq
        
        self.ekf = EKF()
        
        # subscribe and publish
        self.get_logger().info("ekf_gnss node initialized, dt = %f" % self.dt)
        self.gnss_local_sub = self.create_subscription(PoseWithCovarianceStamped, "/gnss_local", self.gnss_cb, qos_profile=qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, qos_profile=qos_profile_sensor_data)
        self.wheel_sub = self.create_subscription(AckermannDriveStamped, "/drive_info_from_nucleo", self.wheel_cb, qos_profile=qos_profile_sensor_data)
        self.ekf_pub = self.create_publisher(PoseWithCovarianceStamped, "/gnss_ekf", 10)
        self.timer = self.create_timer(self.dt, self.ekf_update)

        self.last_yaw = None
        self.curr_w = None

        self.last_x = None
        self.last_y = None 
        self.last_x_covar = None
        self.last_y_covar = None
        self.curr_v = None
        self.wheel_v = None
        self.last_v = None
        self.X_est = None
        self.P_est = np.eye(5)
        self.initStatus_1 = False
        self.initStatus_2 = False

        self.msgLock = threading.Lock()
    

    def imu_cb(self, msg: Imu):
        # calculate the current yaw from orientation quaternion
        q = msg.orientation
        q = np.array([q.w, q.x, q.y, q.z])
        euler = transforms3d.euler.quat2euler(q)
        yaw = euler[2]
        yaw_vel = msg.angular_velocity.z

        #get velocity from dt
        if(not self.initStatus_1):
            self.last_yaw = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, yaw)
            return
        dt = get_Time(msg.header.stamp.sec, msg.header.stamp.nanosec) - get_Time(self.last_yaw.sec, self.last_yaw.nanosec)
        v = msg.linear_acceleration.x*dt

        #instant info of YAW, ANGULAR VELO, and current YAW to be taken from the IMU
        self.last_yaw = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, yaw)
        self.curr_w = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, yaw_vel)
        self.curr_v = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, v)

    def wheel_cb(self, msg: AckermannDriveStamped):
        # calculate the current yaw from orientation quaternion
        speed = msg.drive.speed
        self.wheel_v = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, speed)

    def gnss_cb(self, msg:PoseWithCovarianceStamped):
        # calculate the current x and y position from the gnss data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        #ONLY update position using GPS because thats what it is good at
        self.last_x = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, x)
        self.last_y = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, y)

        self.last_x_covar = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.covariance[0])
        self.last_y_covar = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.covariance[4])

        if(msg.pose.covariance[0] > 0.0025 or msg.pose.covariance[4] > 0.0025):
            self.last_x_covar = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, 1e5*msg.pose.covariance[0])
            self.last_y_covar = Stamped(msg.header.stamp.sec, msg.header.stamp.nanosec, 1e5*msg.pose.covariance[4])

    def checkStatus(self):
        if self.last_yaw is not None and self.last_x is not None and self.last_y is not None:
            self.initStatus_1 = True
        
        if self.curr_v is not None and self.curr_w is not None and self.wheel_v is not None:
            self.initStatus_2 = True
            self.X_est = X(self.last_x.sec, self.last_x.nanosec, self.last_x.val, self.last_y.val, self.last_yaw.val, self.curr_v.val, self.curr_w.val)
            self.get_logger().info("EKF initialized")
    
    def ekf_update(self):
        if(not self.initStatus_1 or not self.initStatus_2):
           self.checkStatus()
           return
        
        # get current time in sec and nanosec
        last_x_est_time = get_Time(self.X_est.sec, self.X_est.nanosec)
        curr_obs_time = get_Time(self.last_x.sec, self.last_x.nanosec)

        dt = curr_obs_time - last_x_est_time

        ud = np.array([[self.wheel_v.val], [self.curr_w.val]])
        x_est = np.array([[self.X_est.x], [self.X_est.y], [self.X_est.yaw], [self.X_est.v], [self.X_est.omega]])

        xEst, PEst = self.ekf.ekf_predict(x_est, self.P_est, ud, self.dt)

        #GNSS ONLY UPDATE
        z_b = np.array([True, True, False, False, False])
        z_covar = np.diag([self.last_x_covar.val, self.last_y_covar.val]) ** 2
        z = np.array([[self.last_x.val], [self.last_y.val]]) 
        xEst, PEst = self.ekf.ekf_partial_update(xEst, PEst, z, z_b ,z_covar)

        #IMU ONLY UPDATE
        z_b = np.array([False, False, True, False, False]) #yaw, velo, yawvelo
        z_covar = np.diag([1e-2]) ** 2
        z = np.array([[self.last_yaw.val]]) 
        xEst, PEst = self.ekf.ekf_partial_update(xEst, PEst, z, z_b ,z_covar)

        quat_to_pub = transforms3d.euler.euler2quat(0,0,xEst[2][0])

        #the time stamps are wrong here UPDATE IT TODO
        self.X_est = X(self.last_x.sec, self.last_x.nanosec, xEst[0][0], xEst[1][0], xEst[2][0], xEst[3][0], xEst[4][0])
        self.P_est = PEst
         
        # publish the estimated position
        msg = PoseWithCovarianceStamped()
        msg.header.stamp.sec = self.last_x.sec
        msg.header.stamp.nanosec = self.last_x.nanosec
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = xEst[0][0]
        msg.pose.pose.position.y = xEst[1][0]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = quat_to_pub[1]
        msg.pose.pose.orientation.y = quat_to_pub[2]
        msg.pose.pose.orientation.z = quat_to_pub[3]
        msg.pose.pose.orientation.w = quat_to_pub[0]
        msg.pose.covariance[0] = PEst[0][0] #x
        msg.pose.covariance[7] = PEst[1][1] #y
        msg.pose.covariance[35] = PEst[2][2] #yaw
        self.ekf_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Ekf_gnss_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()