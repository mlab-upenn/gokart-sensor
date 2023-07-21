#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime
from geometry_msgs.msg import PointStamped
import math

# create a new file
file_name = '/sim_ws/src/pure_pursuit/pure_pursuit/checkpoints/' + str(datetime.now().strftime("%m-%d--%H:%M:%S")) + '.csv'
file = open(file_name, 'w')
file.close()

class ClickPointLoggerNode(Node):
	def __init__(self):
		super().__init__('ClickPointLoggerNode')

		self.subscription_ = self.create_subscription(PointStamped, '/clicked_point', self.callback, 10)

	def callback(self, point_msg):
		x = point_msg.point.x
		y = point_msg.point.y
		# is_turn = input("Is this point for turn (y/N): ")
		# if(is_turn in ['y', 'Y']):
		# 	vel = -100
		# else:
		# 	vel = 100
		vel = 100

		with open(file_name, 'a') as file:
			file.write('%f, %f, %f\n'%(x, y, vel))
			print('Wrote: %f, %f, %f\n'%(x, y, vel))

def main(args=None):
	rclpy.init(args=args)
	print("Logging Clicked Waypoints")
	obj = ClickPointLoggerNode()
	rclpy.spin(obj)

	rrt_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()