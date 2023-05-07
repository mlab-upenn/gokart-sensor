#!/usr/bin/env python3

"""
Copyright (c) 2020 Robotic Arts Industries

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

"""

"""

Author:  Robert Vasquez Zavaleta

"""


import rclpy
from rclpy.node import Node
from ros_imu_bno055.imu_bno055_api import * 
import time
import os

NOT_CALIBRATED = 0x00
FULL_CALIBRATION = 0x01

class CalibrationIMU(Node):

    def __init__(self):


        # Init node
        super().__init__('ros_imu_bno055_node')
        self.node_name = 'ros_imu_bno055_calibration_node'

        # Get ros params
        self.declare_parameter('serial_port')
        self.declare_parameter('operation_mode')
        self.declare_parameter('config_folder')
        self.get_ros_params()

        # Create an IMU instance
        self.bno055 = BoschIMU(port = self.serial_port)
        self.calibration_full_counter = 0

        # create timer
        self.init_calibration() 
        self.timer = self.create_timer(0, self.timer_callback)


    def get_ros_params(self):

        self.serial_port = self.get_parameter('serial_port').value
        self.operation_mode_str = self.get_parameter('operation_mode').value

        switcher = {

            'IMU': IMU,
            'COMPASS': COMPASS,
            'M4G': M4G,
            'NDOF_FMC_OFF': NDOF_FMC_OFF,
            'NDOF': NDOF,
        }

        self.operation_mode = switcher.get(self.operation_mode_str, 'IMU')


    def init_calibration(self):
        self.get_logger().info(f"The IMU will be calibrated to work in {self.operation_mode_str} mode")


    def calibrate_imu(self):

        is_imu_calibrated = NOT_CALIBRATED

        calibration_status, status = self.bno055.calibrate_imu(self.operation_mode)
        
        if status == RESPONSE_OK:

            system_calibration_status = calibration_status[0]
            gyroscope_calibration_status = calibration_status[1]
            accelerometer_calibration_status = calibration_status[2]
            magnetometer_calibration_status = calibration_status[3]
            

            # Calibration for NDOF_FMC_OFF and NDOF             
            if self.operation_mode == NDOF_FMC_OFF or self.operation_mode == NDOF:

                self.get_logger().info("[System: " + str(system_calibration_status) + "]")
                self.get_logger().info(" [Gyroscope: " + str(gyroscope_calibration_status) + "]") 
                self.get_logger().info(" [Accelerometer: " + str(accelerometer_calibration_status) + "]")
                self.get_logger().info(" [Magnetometer: " + str(magnetometer_calibration_status) + "]" )

                if (system_calibration_status == 3 and gyroscope_calibration_status == 3
                and accelerometer_calibration_status == 3 and magnetometer_calibration_status == 3) :

                    self.calibration_full_counter+=1


            # Calibration for IMU
            if self.operation_mode == IMU:

                self.get_logger().info(" [Gyroscope: " + str(gyroscope_calibration_status) + "]") 
                self.get_logger().info(" [Accelerometer: " + str(accelerometer_calibration_status) + "]")

                if (gyroscope_calibration_status == 3 and accelerometer_calibration_status == 3) :
                    self.calibration_full_counter+=1
            

            # Calibration for COMPASS  and M4G
            if self.operation_mode == COMPASS or self.operation_mode == M4G:
                
                self.get_logger().info(" [Accelerometer: " + str(accelerometer_calibration_status) + "]", end = '')
                self.get_logger().info(" [Magnetometer: " + str(magnetometer_calibration_status) + "]" )

                if(accelerometer_calibration_status == 3 and magnetometer_calibration_status == 3) :
                    self.calibration_full_counter+=1


            # When the calibration is FULL three consecutive times, the calibration is assumed to be good
            if self.calibration_full_counter >= 3:

                is_imu_calibrated = FULL_CALIBRATION
                self.get_logger().info("IMU successfully calibrated!")


            time.sleep(1)

        return is_imu_calibrated


    def read_calibration(self):

        calibration, status = self.bno055.get_calibration()

        if status == RESPONSE_OK:
            pass
        else:
            self.get_logger().warn("Unable to read IMU calibration")

        return calibration


    def write_calibration(self, calibration):


        status = self.bno055.set_calibration(calibration)
    
        if status == RESPONSE_OK:
            self.get_logger().info("Calibration successfully written to the IMU")
            self.save_calibration_in_file(calibration)

        else:
            self.get_logger().info("Unable to calibrate the IMU")


    def save_calibration_in_file(self, calibration):

        # Path to this file
        dir_path = os.path.dirname(os.path.realpath(__file__))

        try: 
            binary_file = open(os.path.join(self.get_parameter("config_folder").value, 
                                            self.operation_mode_str + "_calibration"), "wb")
            # binary_file = open(str(dir_path) + "/" + str(self.operation_mode_str) + "_calibration", "wb")
            binary_file.write(calibration)
            binary_file.close()

            self.get_logger().info("Calibration successfully saved in binary file 'calibration'")

        except: 

            self.get_logger().info("Error while saving calibration in file 'calibration'")
        
        

    def read_calibration_from_file(self):

        # Path to this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        
        try:
            binary_file = open(str(dir_path) + "/calibration", "rb")
            calibration_data = binary_file.read()
            binary_file.close()

            self.get_logger().info("The calibration read from the 'calibration' file is: ")
            self.get_logger().info(calibration_data)
        
        except:
            calibration_data = 0
            self.get_logger().info("The file does not exist or the file cannot be read")

        return calibration_data

    def timer_callback(self):
        status = self.calibrate_imu()
        if status == FULL_CALIBRATION:
            calibration_data = self.read_calibration()
            self.write_calibration(calibration_data)
            self.get_logger().info("Calibration has finished successfully. Closing node...")
            rclpy.shutdown()

    # def run(self):

    #     self.init_calibration()
        
    #     while not rospy.is_shutdown():

    #         # IMU calibration
    #         status = self.calibrate_imu()

    #         if status == FULL_CALIBRATION:

    #             # Read calibration and show it by the terminal
    #             calibration_data = self.read_calibration()

    #             # Write the calibration to the IMU and save it to a binary file
    #             self.write_calibration(calibration_data)

    #             rospy.loginfo("Calibration has finished successfully. Closing node...")
    #             rospy.signal_shutdown("")


def main():
    rclpy.init(args=None)
    imuNode = CalibrationIMU()
    rclpy.spin(imuNode)


if __name__ == '__main__':
    # [ros2]
    main()