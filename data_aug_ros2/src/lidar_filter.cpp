#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LidarFilter : public rclcpp::Node
{
  public:
    LidarFilter()
    : Node("lidar_filter"), count_(0)
    {
      std::cout << "dscdsdscds" << std::endl;
      RCLCPP_INFO(this->get_logger(), "I heard:'");
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_lidar", 10);
      // timer_ = this->create_wall_timer(10ms, std::bind(&LidarFilter::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points", 10, std::bind(&LidarFilter::topic_callback, this, _1));
    }

  private:




    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      std::cout << "dscdsdscds" << std::endl;
      // sensor_msgs::msg::PointCloud2 pmg = *msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *in_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ >::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ >::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        /// Pass through filters
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(in_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0, 5);
        pass_x.filter(*cloud_filtered_x);
        sensor_msgs::msg::PointCloud2 out_cloud_filter_x;
        pcl::toROSMsg(*cloud_filtered_x, out_cloud_filter_x);
        publisher_->publish(out_cloud_filter_x);



        // pcl::PassThrough<pcl::PointXYZ> pass_y;
        // pass_y.setInputCloud(cloud_filtered_x);
        // pass_y.setFilterFieldName("y");
        // pass_y.setFilterLimits(y_min, y_max);
        // pass_y.filter(*cloud_filtered_y);

        // pcl::PassThrough<pcl::PointXYZ> pass_z;
        // pass_z.setInputCloud(cloud_filtered_y);
        // pass_z.setFilterFieldName("z");
        // pass_z.setFilterLimits(z_min, z_max);
        // pass_z.filter(*cloud_filtered_z);

    }

    // void timer_callback()
    // {
    // }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    double lat_old_, lon_old_;
    double yaw_old_=0.0, yaw_publish_=0.0;
    bool switch_ = false;
    bool jump_ = false;
    size_t count_;
    int ct_=0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}