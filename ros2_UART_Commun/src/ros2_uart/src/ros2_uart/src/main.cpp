#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "serial/serial.h"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      // publisher_ = this->create_publisher<std_msgs::msg::String>(
      // "points2", 10);
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      // Define port, baud rate, and update frequency in ms (exaple 1000 = 1 second) of UART connector
      std::string port ("/media/ttyUSB0");
      int baud = 446580;
      int update_frequency = 100;

      // Defines Serial Port
      serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(update_frequency));

      if(my_serial.isOpen())
        std::cout << "The USB Port is opened." << std::endl;

      std::string uart_write_message (msg->data.c_str());
      my_serial.write(uart_write_message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}