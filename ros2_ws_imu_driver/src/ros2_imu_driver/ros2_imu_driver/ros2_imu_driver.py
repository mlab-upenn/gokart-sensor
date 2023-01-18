#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial
from imu_msgs.msg import Imu


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
                # assign elements 1-2 and 5 to 9 to the ROS message
                msg.accel_x = float(data[0])
                msg.accel_y = float(data[1])
                msg.accel_z = float(data[2])
                msg.gyro_x = float(data[3])
                msg.gyro_y = float(data[4])
                msg.gyro_z = float(data[5])
                msg.mag_x = float(data[6])
                msg.mag_y = float(data[7])
                msg.mag_z = float(data[8])

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