#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <math.h>
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/use", 10);
      gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/navsatfix/use", 10);
      timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback, this));
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&MinimalPublisher::imu_callback, this, _1));
      gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsatfix", 10, std::bind(&MinimalPublisher::gnss_callback, this, _1));
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu message = *msg;
        std::array<double, 9> covar = msg->orientation_covariance;
        std::array<double, 9> covar_acc = msg->orientation_covariance;
        std::array<double, 9> covar_angv = msg->orientation_covariance;

        covar[0] = 2*1e-9;
        covar[4] = 2*1e-9;
        covar[8] = 2*1e-9;

        covar_acc[0] = 1;
        covar_acc[4] = 1;
        covar_acc[8] = 1;

        covar_angv[0] = 2*1e-9;
        covar_angv[4] = 2*1e-9;
        covar_angv[8] = 2*1e-9;

        message.orientation_covariance = covar;
        message.angular_velocity_covariance = covar_angv;
        message.linear_acceleration_covariance = covar_acc;

        publisher_->publish(message);
    }

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto message = *msg;
        switch_ = true;
        std::array<double, 9> gps_covar = msg->position_covariance;
        double lat = msg->latitude;
        double lon = msg->longitude;
        if(abs(gps_covar[0])>1000 || abs(lat) > 180 || abs(lon) >180)
        {
            lat = lat_old_;
            lon = lon_old_;
            std::array<double, 9> gps_covar_new;
            gps_covar_new[0]= 1e9;
            gps_covar_new[1]= 0.0;
            gps_covar_new[2]= 0.0;
            gps_covar_new[3]= 0.0;
            gps_covar_new[4]= 1e9;
            gps_covar_new[5]= 0.0;
            gps_covar_new[6]= 0.0;
            gps_covar_new[7]= 0.0;
            gps_covar_new[8]= 1e9;
            gps_covar = gps_covar_new;
            ct_+=1;
        }
        else
        {
            lat_old_ = lat;
            lon_old_ = lon;
        }
        message.latitude = lat;
        message.longitude = lon;
        message.position_covariance = gps_covar;
        publish_navsat_ = message;
        gps_publisher_->publish(publish_navsat_);

    }

    void timer_callback()
    {
      // if(switch_) gps_publisher_->publish(publish_navsat_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::NavSatFix publish_navsat_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    
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
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}