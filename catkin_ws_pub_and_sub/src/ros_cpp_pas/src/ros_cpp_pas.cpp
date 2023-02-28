#include <ros/ros.h>
#include <std_msgs/String.h>


ros::Publisher pub;


void ros_cpp_pas(const std_msgs::String &msg)
{
	pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_cpp_pas");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("input", 10, ros_cpp_pas);
	pub = nh.advertise<std_msgs::String>("output", 10);
	ros::spin();
}