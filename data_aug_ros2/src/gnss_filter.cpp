#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
      gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/navsatfix/drop", 10);
      gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/navsatfix", 10, std::bind(&MinimalPublisher::gps_callback, this, _1));
      unreliable_flag = false;
      unreliable_count = 0;
    }

  private:

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    size_t count_;
    bool unreliable_flag;
    int unreliable_count;

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        auto message = *msg;
        std::array<double, 9> gps_covar = msg->position_covariance;
        double lat = msg->latitude;
        double lon = msg->longitude;
        if(abs(gps_covar[0])>1000 || abs(lat) > 180 || abs(lon) >180)
        {
            if(!unreliable_flag)
            {
                int sec = msg->header.stamp.sec;
                int nanosec = msg->header.stamp.nanosec;
                RCLCPP_INFO(this->get_logger(), "GPS data start to be not reliable at time %d.%d", sec, nanosec);

                unreliable_flag = true;
                unreliable_count = 0;
            }
            unreliable_count++;
        }
        else
        {
            if(unreliable_flag)
            {
                int sec = msg->header.stamp.sec;
                int nanosec = msg->header.stamp.nanosec;
                RCLCPP_INFO(this->get_logger(), "GPS data end to be not reliable at time %d.%d, count is %d", sec, nanosec, unreliable_count);

                unreliable_count = 0;
                unreliable_flag = false;
            }
            message.latitude = lat;
            message.longitude = lon;
            gps_publisher_->publish(message);
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}