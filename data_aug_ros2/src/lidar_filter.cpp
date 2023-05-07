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
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    LidarFilter(): Node("lidar_filter"), count_(0)
    {
      // std::cout << "dscdsdscds" << std::endl;
      // RCLCPP_INFO(this->get_logger(), "I heard:'");
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_lidar", 10);
      // timer_ = this->create_wall_timer(10ms, std::bind(&LidarFilter::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points", rclcpp::SensorDataQoS(), std::bind(&LidarFilter::topic_callback, this, std::placeholders::_1));
    }

  private:

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // std::cout << "dscdsdscds" << std::endl;
        // RCLCPP_INFO(this->get_logger(), "I heard:'");
        // sensor_msgs::msg::PointCloud2 pmg = *msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *in_cloud);

        //TRY
        // Define the rotation matrix
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        float theta = -M_PI/2;  // rotation angle in radians
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
        // Apply the rotation to the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*in_cloud, *cloud_rotated, transform);

        // Create the Voxel Grid filter object
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud (cloud_rotated);
        voxel_filter.setLeafSize (0.06f, 0.06f, 0.06f); // set the voxel size
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter (*voxel_cloud);  // apply the filter and save the output to the original point cloud object

        /// Pass through filters
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(voxel_cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(1.5, 4);
        pass_x.filter(*cloud_filtered_x);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud_filtered_x);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-3, -0.0);
        pass_z.filter(*cloud_filtered_z);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered_z);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-3, 3);
        pass_y.filter(*cloud_filtered_y);

        /// Plane Segmentation
        pcl::PointCloud<pcl::PointXYZ >::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered_y));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.05);
        ransac.computeModel();
        std::vector<int> inliers_indicies;
        ransac.getInliers(inliers_indicies);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered_y, inliers_indicies, *plane);

        /// Statistical Outlier Removal
        // pcl::PointCloud<pcl::PointXYZ >::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(plane);
        // sor.setMeanK (50);
        // sor.setStddevMulThresh (1);
        // sor.filter (*plane_filtered);

        /// Extract non-inliers to get the remaining points
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        inliers->indices = inliers_indicies;
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered_y);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*remaining);

        // Euclidean cluster extraction
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(remaining);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(10);
        ec.setSearchMethod(tree);
        ec.setInputCloud(remaining);
        ec.extract(cluster_indices);
        std::cout << cluster_indices.size() << std::endl; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cluster_cloud->points.push_back (remaining->points[*pit]);
        }


        sensor_msgs::msg::PointCloud2 out_cloud_filter_x;
        pcl::toROSMsg(*cluster_cloud, out_cloud_filter_x);
        out_cloud_filter_x.header = (*msg).header;
        publisher_->publish(out_cloud_filter_x);




        // // Create the Euclidean Cluster Extraction object
        // pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
        // cluster_extractor.setInputCloud (cloud_filtered_z);
        // cluster_extractor.setClusterTolerance (0.1); // set the cluster tolerance to 0.1 meters
        // cluster_extractor.setMinClusterSize (50);   // set the minimum cluster size to 50 points
        // cluster_extractor.setMaxClusterSize (1000); // set the maximum cluster size to 10000 points
        // std::vector<pcl::PointIndices> cluster_indices;
        // cluster_extractor.extract (cluster_indices); // perform clustering and save the output to a vector of point indices

        // // Create a new point cloud for each cluster and save to a PCD file
        // // int i = 0;

        // cluster_cloud->width = cluster_cloud->points.size ();
        // cluster_cloud->height = 1;
        // cluster_cloud->is_dense = true;
        // sensor_msgs::msg::PointCloud2 out_cloud_filter_x;
        // // out_cloud_filter_x.header = (*msg).header;
        // pcl::toROSMsg(*cloud_filtered_z, out_cloud_filter_x);
        // out_cloud_filter_x.header = (*msg).header;
        // publisher_->publish(out_cloud_filter_x);
        // //TRY ENDS

        // sensor_msgs::msg::PointCloud2 out_cloud_filter_x;
        // pcl::toROSMsg(*cloud_filtered_x, out_cloud_filter_x);
        // publisher_->publish(out_cloud_filter_x);

        // sensor_msgs::msg::PointCloud2 out_cloud_filter_x;
        // pcl::toROSMsg(*cloud_filtered_z, out_cloud_filter_x);
        // publisher_->publish(out_cloud_filter_x);



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