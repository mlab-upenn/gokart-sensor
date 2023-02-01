#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>


ros::Publisher pub;


void pcl_preprocessing(const sensor_msgs::PointCloud2ConstPtr &msg_in)
{
	// initialize new pointclouds for removal of points behind the kart
	pcl::PCLPointCloud2 *point_cloud_original = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr point_cloud_original_Ptr(point_cloud_original);
	pcl::PCLPointCloud2 point_cloud_processed;
	pcl::PCLPointCloud2 point_cloud_processed_final;
	pcl_conversions::toPCL(*msg_in, *point_cloud_original);

	// values for bounding box
	double x_min = -0.0;
	double x_max = 60.0;
	double y_min = -60.0;
	double y_max = 60.0;
	double z_min = -20.0;
	double z_max = 20.0;

	// remove points behind go-kart
	pcl::CropBox<pcl::PCLPointCloud2> cropBoxFilter_rear (true);
	cropBoxFilter_rear.setInputCloud (point_cloud_original_Ptr);
	cropBoxFilter_rear.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
	cropBoxFilter_rear.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
	cropBoxFilter_rear.filter(point_cloud_processed);


	// initialize pointer to first processed point cloud
	pcl::PCLPointCloud2::Ptr input (new pcl::PCLPointCloud2 (point_cloud_processed));

	// update values for second bounding box
	x_min = -1.0;
	x_max = 1.8;
	y_min = -1.0;
	y_max = 1.0;
	z_min = -20.0;
	z_max = 20.0;

	// remove front LiDAR in pointcloud
	pcl::CropBox<pcl::PCLPointCloud2> cropBoxFilter_front_sensors;
	cropBoxFilter_front_sensors.setInputCloud(input);
	cropBoxFilter_front_sensors.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
	cropBoxFilter_front_sensors.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
	cropBoxFilter_front_sensors.setNegative(true);
	cropBoxFilter_front_sensors.filter(point_cloud_processed_final);

	// convert back to ROS sensor_msgs
	sensor_msgs::PointCloud2 msg_out;
	pcl_conversions::fromPCL(point_cloud_processed_final, msg_out);

	// publish on /ouster/points_filtered topic
	pub.publish(msg_out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_preprocessing");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/ouster/points", 10, pcl_preprocessing);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/ouster/points_filtered", 10);
	ros::spin();
}