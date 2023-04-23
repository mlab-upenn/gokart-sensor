#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped


class kart_interface(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive_command_to_nucleo', 10)
        self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)


    def teleop_callback(self, twist_msg):
        ego_requested_speed = twist_msg.linear.x
        ego_requested_angle = twist_msg.angular.z

        if ego_requested_angle > 0.0:
            ego_requested_angle = 0.3
        elif ego_requested_angle < 0.0:
            ego_requested_angle = -0.3
        else:
            ego_requested_angle = 0.0
        
        if ego_requested_speed >0.2:
            ego_requested_speed = 0.2
        elif ego_requested_speed<-0.2:
            ego_requested_speed = -0.2
        else:
            ego_requested_speed =0.0
        
        command_to_publish  = AckermannDriveStamped()
        command_to_publish.header.stamp = self.get_clock().now().to_msg()
        command_to_publish.drive.steering_angle = ego_requested_angle
        command_to_publish.drive.speed = ego_requested_speed
        self.publisher_.publish(command_to_publish)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = kart_interface()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

