#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gnss_msgs.msg import GNSS

import serial


class GPS_Publisher(Node):

    def __init__(self):
        super().__init__('gps_publisher_node')
        self.gps_publisher_ = self.create_publisher(GNSS, '/gnss', 10)
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=460800)

        self.lat = self.lon = self.heading = -1.0
        self.gpgga_flag = False
        self.gphdt_flag = False

        self.parse_gps_data()

    def parse_gps_data(self):

        while True:
            data = str(self.ser.readline())
            gpgga_idx = data.find("$GPGGA")
            gphdt_idx = data.find("$GPHDT")
            if (gpgga_idx != -1):  # gpgga data in this string
                gpgga_str = data.split("$GPGGA", maxsplit=1)[1]
                gpgga_data = gpgga_str.split(",")[1:]
                self.lat = gpgga_data[1]
                if (self.lat != ''):
                    self.lat = float(self.lat)
                    self.lat = self.lat//100 + (self.lat%100)/60
                else:
                    self.gpgga_flag = False
                    continue
                if (gpgga_data[2] == 'S'):
                    self.lat *= -1
                self.lon = gpgga_data[3]
                if (self.lon != ''):
                    self.lon = float(self.lon)
                    self.lon = self.lon//100 + (self.lon%100)/60
                else:
                    self.gpgga_flag = False
                    continue
                if (gpgga_data[4] == 'W'):
                    self.lon *= -1
                self.gpgga_flag = True
            print("Lat: ", self.lat)
            print("Lon: ", self.lon)

            if (gphdt_idx != -1):  # gphdt data in this string
                gphdt_str = data.split("$GPHDT", maxsplit=1)[1]
                gphdt_data = gphdt_str.split(",")[1:]
                self.heading = gphdt_data[0]
                if (self.heading == ''):  # no heading data being received
                    self.gphdt_flag = False
                    # continue
                else:
                    self.heading = 90 - float(self.heading)
                    if(self.heading < -180):
                        self.heading += 360
                    self.gphdt_flag = True

            if (self.gpgga_flag or self.gphdt_flag):  # TODO: MAKE AND
                gps_data = GNSS()
                gps_data.latitude = self.lat
                gps_data.longitude = self.lon
                gps_data.heading = self.heading
                gps_data.success = True
            else:
                gps_data = GNSS()
                self.gps_publisher_.publish(gps_data)


            print("_-"*10)
            self.gps_publisher_.publish(gps_data)

    # except Exception as e:
    #     print("Exception is GPS Data: %s" % e)
    # finally:
    #     print("\n\nClosing serial port safely")
    #     self.ser.close()x


def main(args=None):
    rclpy.init(args=args)
    obj = GPS_Publisher()
    rclpy.spin(obj)
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
