#!/usr/bin/env python
import serial
import rospy
from gnss_driver.msg import lat_long_heading

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=460800)

lat = lon = heading = -1.0
gpgga_flag = False
gphdt_flag = False

# while True:
#     data = str(ser.readline())
#     gpgga_idx = data.find("$GPGGA")
#     gphdt_idx = data.find("$GPHDT")
#     if (gpgga_idx != -1):  # gpgga data in this string
#         gpgga_str = data.split("$GPGGA", maxsplit=1)[1]
#         gpgga_data = gpgga_str.split(",")[1:]
#         lat = gpgga_data[1]
#         if (lat != ''):
#             lat = float(lat)
#             lat = lat // 100 + (lat % 100) / 60
#         else:
#             gpgga_flag = False
#             continue
#         if (gpgga_data[2] == 'S'):
#             lat *= -1
#         lon = gpgga_data[3]
#         if (lon != ''):
#             lon = float(lon)
#             lon = lon // 100 + (lon % 100) / 60
#         else:
#             gpgga_flag = False
#             continue
#         if (gpgga_data[4] == 'W'):
#             lon *= -1
#         gpgga_flag = True
#     print("Lat: ", lat)
#     print("Lon: ", lon)
#
#     if (gphdt_idx != -1):  # gphdt data in this string
#         gphdt_str = data.split("$GPHDT", maxsplit=1)[1]
#         gphdt_data = gphdt_str.split(",")[1:]
#         heading = gphdt_data[0]
#         if (heading == ''):  # no heading data being received
#             gphdt_flag = False
#             # continue
#         else:
#             heading = 90 - float(heading)
#             if (heading < -180):
#                 heading += 360
#             gphdt_flag = True
#             print("Heading: " , heading)
#
#     print("_-" * 10)

def GPS_publisher():
    lat = lon = heading = -1.0
    gpgga_flag = False
    gphdt_flag = False
    pub = rospy.Publisher('GNSS_LatLong', lat_long_heading, queue_size=10)
    msg = lat_long_heading()
    rospy.init_node('gnss_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = str(ser.readline())
        gpgga_idx = data.find("$GPGGA")
        gphdt_idx = data.find("$GPHDT")
        if (gpgga_idx != -1):  # gpgga data in this string
            gpgga_str = data.split("$GPGGA", maxsplit=1)[1]
            gpgga_data = gpgga_str.split(",")[1:]
            lat = gpgga_data[1]
            if (lat != ''):
                lat = float(lat)
                lat = lat // 100 + (lat % 100) / 60
            else:
                lat = 0.0
                gpgga_flag = False
                continue
            if (gpgga_data[2] == 'S'):
                lat *= -1
            lon = gpgga_data[3]
            if (lon != ''):
                lon = float(lon)
                lon = lon // 100 + (lon % 100) / 60
            else:
                lon = 0.0
                gpgga_flag = False
                continue
            if (gpgga_data[4] == 'W'):
                lon *= -1
            gpgga_flag = True
        print("Lat: ", lat)
        print("Lon: ", lon)

        if (gphdt_idx != -1):  # gphdt data in this string
            gphdt_str = data.split("$GPHDT", maxsplit=1)[1]
            gphdt_data = gphdt_str.split(",")[1:]
            heading = gphdt_data[0]
            if (heading == ''):  # no heading data being received
                heading = 0.0
                gphdt_flag = False
                # continue
            else:
                heading = 90 - float(heading)
                if (heading < -180):
                    heading += 360
                gphdt_flag = True
                print("Heading: ", heading)

        print("_-" * 10)
        msg.latitude = lat
        msg.longitude = lon
        msg.heading = heading
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        GPS_publisher()
    except rospy.ROSInterruptException:
        pass