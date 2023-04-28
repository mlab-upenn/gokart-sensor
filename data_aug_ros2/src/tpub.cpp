#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class MinimalSubscriber : public rclcpp::Node
{
  public:

    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
      ("/ouster/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg->height); //
      std::cout << "Complete ROS cloud is: " << msg.get() << std::endl;      

    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  std::cout << "Inside main function now";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}