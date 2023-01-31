#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Imu
import serial
import time

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

def talker():
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('imu_sensor_msgs', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        start_time = time.time()

        data = ser.readline().decode('utf-8')
        print(data)
        # multiple elements removed from the string
        data = data.replace('\n', '').replace('\r', '').replace('\x00', '')
        print(data)
        # data split at every comma
        data = data.split(',')
        print(data)

        # only publish message if imu data is not empty
        if data != ['']:
            # initialize message from custom IMU message
            msg = Imu()
            # assign elements 1-2 and 5 to 9 to the ROS message
            msg.header.stamp = rospy.Time.now()


            try:
                msg.linear_acceleration.x = float(data[0]) / 1000 * 9.81
            except (ValueError, IndexError):
                msg.linear_acceleration.x = msg_hist.linear_acceleration.x

            try:
                msg.linear_acceleration.y = -float(data[1])/1000*9.81
            except (ValueError, IndexError):
                msg.linear_acceleration.y = msg_hist.linear_acceleration.y

            try:
                msg.linear_acceleration.z = float(data[2])/1000*9.81
            except (ValueError, IndexError):
                msg.linear_acceleration.z = msg_hist.linear_acceleration.z

            try:
                msg.angular_velocity.x = (float(data[3])*3.14)/(180*1000)
            except (ValueError, IndexError):
                msg.angular_velocity.x = msg_hist.angular_velocity.x

            try:
                msg.angular_velocity.y = (float(data[4])*3.14)/(180*1000)
            except (ValueError, IndexError):
                msg.angular_velocity.y = msg_hist.angular_velocity.y
            try:
                msg.angular_velocity.z = (float(data[5])*3.14)/(180*1000)
            except (ValueError, IndexError):
                msg.angular_velocity.z = msg_hist.angular_velocity.z

            msg_hist = msg
            print("--- %s seconds ---" % (time.time() - start_time))
            # publish message
            pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
