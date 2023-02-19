#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <iostream>


ros::Publisher pub;


void pcl_preprocessing(const darknet_ros_msgs::bounding_boxes &msg_in)
{
	
	// define intrinsic camera parameters
	double fx = 1442.193090;
	double fy = 1440.905704;
	double cx = 943.936435;
	double cy = 542.578706;

	// define cone parameters
	double cone_height = 0.7;

	std::cout << "Works" << std::endl;

	// height_pixel = msg_in.bounding_boxes.
	
	
	
	
	
	// // mark position in LiDAR frame
	// visualization_msgs::Marker marker;
	// marker.header.frame_id = "/os_sensor";
	// marker.header.stamp = ros::Time::now();
	// marker.ns = "my_namespace";
	// marker.id = 0;
	// marker.type = 3;
	// marker.action = 0;
	// marker.pose.position.x = 0;
	// marker.pose.position.y = 0;
	// marker.pose.position.y = 0;
	// marker.pose.orientation.x = 0.0;
    // 	marker.pose.orientation.y = 0.0;
    // 	marker.pose.orientation.z = 0.0;
    // 	marker.pose.orientation.w = 1.0;
    // 	marker.scale.x = 0.32;
    // 	marker.scale.y = 0.32;
    // 	marker.scale.z = 0.7;
    // 	marker.color.r = 0.0f;
    // 	marker.color.g = 1.0f;
    // 	marker.color.b = 0.0f;
    // 	marker.color.a = 1.0;
    // 	marker.lifetime = ros::Duration();
	
	// pub.publish(marker);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pnp");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, pcl_preprocessing);
	pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
	ros::spin();
}
