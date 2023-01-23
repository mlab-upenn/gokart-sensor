#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // Define port, baud rate, and update frequency in ms (exaple 1000 = 1 second) of UART connector
      std::string port ("/dev/ttyUSB0");
      int baud = 9600;
      int update_frequency = 500;

      // Defines Serial Port
      serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(update_frequency));

      std::string message = my_serial.read();
      std::cout << my_serial.read(15) << std::endl;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
      // publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
