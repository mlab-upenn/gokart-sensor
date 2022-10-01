#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void cropImageReceived(const sensor_msgs::Image& msg)
{
        sensor_msgs::Image msg_out;
        cout << "Hello!"<< endl;

	cout << "OpenCV version : " << CV_VERSION << endl;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "camera_preprocessing");
        ros::NodeHandle nh;

	cout << "OpenCV version : " << CV_VERSION << endl;

        ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/port_1/camera/image_cropped", 1000);

        ros::Subscriber sub = nh.subscribe("/port_1/camera/image_raw", 1000, &cropImageReceived);

        ros::spinOnce();


        return 0;
}

