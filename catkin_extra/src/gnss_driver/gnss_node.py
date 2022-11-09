#!/usr/bin/env python
import serial
import rospy
from gnss_driver.msg import lat_long_heading
import utm

# set serial port for UART connector GNSS
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=460800)

# look for inits here https://epsg.io/103516
init_wgs = 'epsg:4326'
init_bng = 'epsg:3855'

# Set origin of xy coordinates

lat_orig = 39.941766
lon_orig = -75.198820

x_orig, y_orig, zone_orig, ut_orig = utm.from_latlon(lat_orig, lon_orig)

lat = lon = heading = -1.0
gpgga_flag = False
gphdt_flag = False
lat_history = lon_history = heading_history = lon_orientation_history = lat_orientation_history = lat_orientation = lon_orientation = 0
lat_raw_history = lon_raw_history = 0


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
        print(data)
        gpgga_idx = data.find("$GPGGA")
        gphdt_idx = data.find("$GPHDT")
        if (gpgga_idx != -1):  # gpgga data in this string
            gpgga_str = data.split("$GPGGA", maxsplit=1)[1]
            gpgga_data = gpgga_str.split(",")[1:]
            try:
                lat = gpgga_data[1]
                lat_raw_history = lat
            except IndexError:
                lat = lat_raw_history
            if (lat != ''):
                try:
                    lat = float(lat)
                    lat = lat // 100 + (lat % 100) / 60
                    lat_history = lat
                except ValueError:
                    lat = lat_history
            else:
                lat = 0.0
                lat_history = lat
                gpgga_flag = False
                continue
            try:
                lat_orientation = gpgga_data[4]
                lat_orientation_history = lat_orientation
            except IndexError:
                lat_orientation = lat_orientation_history
            if (lat_orientation == 'S'):
                lat *= -1
            try:
                lon = gpgga_data[3]
                lon_raw_history = lon
            except IndexError:
                lon = lon_raw_history
            if (lon != ''):
                try:
                    lon = float(lon)
                    lon = lon // 100 + (lon % 100) / 60
                    lon_history = lon
                except ValueError:
                    lon = lon_history
            else:
                lon = 0.0
                lon_history = lon
                gpgga_flag = False
                continue

            try:
                lon_orientation = gpgga_data[4]
                lon_orientation_history = lon_orientation
            except IndexError:
                lon_orientation = lon_orientation_history
            if (lon_orientation == 'W'):
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
                heading_history = heading
                # continue
            else:
                try:
                    heading = 90 - float(heading)
                    if (heading < -180):
                        heading += 360
                    gphdt_flag = True
                    print("Heading: ", heading)
                    heading_history = heading
                except ValueError:
                    heading = heading_history

        x, y, zone, ut = utm.from_latlon(lat,lon)

        print("X: ", x - x_orig)
        print("Y: ", y - y_orig)

        print("_-" * 10)

        msg.latitude = lat
        msg.longitude = lon
        msg.heading = heading
        msg.x = (x - x_orig)
        msg.y = (y - y_orig)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        GPS_publisher()
    except rospy.ROSInterruptException:
        pass
