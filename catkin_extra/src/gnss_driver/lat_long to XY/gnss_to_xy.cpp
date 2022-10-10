#include <math.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gokart_msgs/msg/gnss_data.hpp"

#define radians(val) (val * M_PI / 180.0)


using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class GNSS_to_XYZ : public rclcpp::Node {
private:
    void timer_callback() { xy_pub->publish(xy_msg); }

    vector<double> gnss_to_cartesian(
            double point_lat, double point_lon, double radius_north, double radius_east) {
        double point_centered[2] = {point_lat - base_point_gps_[0], point_lon - base_point_gps_[1]};

        double x = radians(point_centered[0]) * radius_north;
        double y = radians(point_centered[1]) * radius_east;

        vector<double> cartesian_xy{y, x};
        return cartesian_xy;
    }

    vector<double> get_earth_radius_at_latitude(double latitude) {
        // constants for from GPS Hector plugin
        double EQUATORIAL_RADIUS = 6378137.0;
        double FLATTENING = 1.0 / 298.257223563;
        double ECCENTRICITY2 = 2.0 * FLATTENING - pow(FLATTENING, 2.0);

        // calculate earth radius from GPS Hector plugin
        double base_point_latitude_ = radians(latitude);
        double temp_ = 1.0 / (1.0 - ECCENTRICITY2 * pow(sin(base_point_latitude_), 2.0));
        double prime_vertical_radius_ = EQUATORIAL_RADIUS * sqrt(temp_);
        double radius_north = prime_vertical_radius_ * (1.0 - ECCENTRICITY2) * temp_;
        double radius_east = prime_vertical_radius_ * cos(base_point_latitude_);

        vector<double> radii{radius_north, radius_east};
        return radii;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr data) {
        tf2::Quaternion q(
                data->orientation.x, data->orientation.y, data->orientation.z, data->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //    printf("Quat w: %f;\tYaw: %f",data->orientation.w, yaw);
        xy_msg.pose.orientation.w = data->orientation.w;
    }

    void heading_callback(const nav_msgs::msg::Odometry::ConstSharedPtr data) {
        xy_msg.pose.orientation = data->pose.pose.orientation;
    }


    void gps_callback(
            const gokart_msgs::msg::GNSSData::ConstSharedPtr gps_msg)  // use this for sim, odom
    {
        current_stamp_ = gps_msg->header.stamp;
        if (gps_msg->success) {
            double latitude = gps_msg->latitude;
            double longitude = gps_msg->longitude;

            printf("Lat: %f; Lon: %f", latitude, longitude);
//    double heading = gps_msg->heading;
            // double altitude             =       gps_msg->altitude;


            vector<double> radii = get_earth_radius_at_latitude(latitude);
            vector<double> cartesian_xy = gnss_to_cartesian(latitude, longitude, radii[0], radii[1]);


            //    auto xy_msg = geometry_msgs::msg::PoseStamped();
            xy_msg.pose.position.x = cartesian_xy[0];
            xy_msg.pose.position.y = cartesian_xy[1];
            // TODO:Make this configurable
            xy_msg.pose.position.z = 0.1396983922541405;

            xy_msg.header.stamp = current_stamp_;

            // TODO: Define a yaml file for the topic names, and frame id
            xy_msg.header.frame_id = "map";
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr xy_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<gokart_msgs::msg::GNSSData>::ConstSharedPtr gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr heading_sub;

    // global parameters
    double base_point_gps_[2] = {40.437265, -86.945105};
    rclcpp::Time current_stamp_;

    geometry_msgs::msg::PoseStamped xy_msg;

public:
    GNSS_to_XYZ() : Node("gnss_to_xyz") {
        xy_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_xy", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&GNSS_to_XYZ::timer_callback, this));
        gps_sub = this->create_subscription<gokart_msgs::msg::GNSSData>(
                "/gnss", 10, std::bind(&GNSS_to_XYZ::gps_callback, this, _1));
        //    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        //      "/imu", 10, std::bind(&GNSS_to_XYZ::imu_callback, this, _1));
//    heading_sub = this->create_subscription<nav_msgs::msg::Odometry>(
//      "/ground_truth", 10, std::bind(&GNSS_to_XYZ::heading_callback, this, _1));
    }


    ~GNSS_to_XYZ() {}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSS_to_XYZ>());
    rclcpp::shutdown();
    return 0;
}
