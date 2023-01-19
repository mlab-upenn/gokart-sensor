#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu


class IMU_ROS_driver(Node):

    def __init__(self):
        super().__init__('IMU_ROS_driver')
        self.publisher_ = self.create_publisher(Imu, 'topic', 10)
        # define serial port with address and baudrate
        self.serial = serial.Serial('/dev/ttyUSB0', 115200)
        self.UART_from_IMU()

    def UART_from_IMU(self):

        while True:
            # read data from serial port and convert to string, decode() is faster than str()
            data = self.serial.readline().decode('utf-8')
            # multiple elements removed from the string
            data = data.replace('\n','').replace('\r','').replace('\x00','')
            # data split at every comma
            data = data.split(',')

            # only publish message if imu data is not empty
            if data != ['']:
                # initialize message from custom IMU message
                msg = Imu()
                # assign aceeleration
                # convert from mg to m/s^2
                msg.linear_acceleration.x = float(data[0])/1000*9.81
                msg.linear_acceleration.y = float(data[1])/1000*9.81
                msg.linear_acceleration.z = float(data[2])/1000*9.81
                # assign angular velocity
                # convert from °/s to rad/s
                msg.angular_velocity.x = float(data[3])*3.14/180
                msg.angular_velocity.y = float(data[4])*3.14/180
                msg.angular_velocity.z = float(data[5])*3.14/180
                # assign orientation
                msg.orientation.x = float(data[6])
                msg.orientation.y = float(data[7])
                msg.orientation.z = float(data[8])
                # define quternion w value
                msg.orientation.w = 1 - (msg.orientation.x**2 + msg.orientation.y**2 + msg.orientation.z**2)
                # ROS header
                msg.header.stamp = self.get_clock().now().to_msg()

                #publish message
                self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    obj = IMU_ROS_driver()
    rclpy.spin(obj)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()