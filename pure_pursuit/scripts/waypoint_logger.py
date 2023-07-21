#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime
from nav_msgs.msg import Odometry
import math
import os
import threading

# create a new file
file_name = '/sim_ws/src/pure_pursuit/pure_pursuit/checkpoints/' + str(datetime.now().strftime("%m-%d--%H:%M:%S")) + '.csv'
file = open(file_name, 'w')
file.close()

class WaypointLoggerNode(Node):
	def __init__(self):
		super().__init__('WaypointLoggerNode')

		self.subscription_ = self.create_subscription(Odometry, '/ego_racecar/odom', self.callback, 10)
		self.velocity = 6.0 
		self.x = self.y = self.theta = None

	def logger(self):
		inp = input("Press y/Y to enter the point: ")
		if(inp in ['y', 'Y']): # add point
			with open(file_name, 'a') as file:
				file.write('%f, %f, %f, %f\n'%(self.x, self.y, self.theta, self.velocity))
				print('Wrote: %f, %f, %f, %f\n'%(self.x, self.y, self.theta, self.velocity))
		elif(inp in ['v', 'V']): # change velocity
			vel = input("Enter new speed: ")
			self.velocity = float(vel)
		elif(inp in ['d', 'D']): # delete last line
			with open(file_name, "r") as f:
				lines=f.readlines()
				lines=lines[:-1] # deleting last line

			with open(file_name, 'w') as f:
				for i in lines:
					f.write(i)
				print("Successfully deleted the last point")
		elif(inp in ['q', 'Q']): # quit
			save = input("Do you want to save your file(y/n): ")
			if(save in ['n','N']):
				os.remove(file_name)
			exit()


	def callback(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		
		euler = self.euler_from_quaternion(data.pose.pose.orientation.x, 
										   data.pose.pose.orientation.y, 
										   data.pose.pose.orientation.z, 
										   data.pose.pose.orientation.w)

		self.theta = euler[2]

 
	def euler_from_quaternion(self, x, y, z, w):
			"""
			Convert a quaternion into euler angles (roll, pitch, yaw)
			roll is rotation around x in radians (counterclockwise)
			pitch is rotation around y in radians (counterclockwise)
			yaw is rotation around z in radians (counterclockwise)
			"""
			t0 = +2.0 * (w * x + y * z)
			t1 = +1.0 - 2.0 * (x * x + y * y)
			roll_x = math.atan2(t0, t1)
		 
			t2 = +2.0 * (w * y - z * x)
			t2 = +1.0 if t2 > +1.0 else t2
			t2 = -1.0 if t2 < -1.0 else t2
			pitch_y = math.asin(t2)
		 
			t3 = +2.0 * (w * z + x * y)
			t4 = +1.0 - 2.0 * (y * y + z * z)
			yaw_z = math.atan2(t3, t4)
		 
			return [roll_x, pitch_y, yaw_z] # in radians


def spin_node(obj):
	rclpy.spin(obj)


def input_thread(obj):
	while(True):
		obj.logger()

def main(args=None):
	rclpy.init(args=args)
	obj = WaypointLoggerNode()
	t1 = threading.Thread(target=spin_node, args=(obj,))
	t2 = threading.Thread(target=input_thread, args=(obj,))
  
	# starting thread 1
	t1.start()
	# starting thread 2
	t2.start()
	t2.join()
	rclpy.shutdown()


if __name__ == '__main__':
	main()