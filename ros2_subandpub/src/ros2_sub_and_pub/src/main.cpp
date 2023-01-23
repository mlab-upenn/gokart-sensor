#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
using std::placeholders::_1;

class pub_and_sub : public rclcpp::Node
{
public:
  pub_and_sub()
  : Node("pub_and_sub_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "points", 10, std::bind(&pub_and_sub::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>(
    "points2", 10);
  }
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "1");
    // publisher_->publish(*msg);
    std::cout<<"test\n";
  }

  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pub_and_sub>());
  rclcpp::shutdown();
  return 0;
}


// class pub_and_sub : public rclcpp::Node
// {
// public:
//   pub_and_sub()
//   : Node("pub_and_sub_node")
//   {
//     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "points", 10, std::bind(&pub_and_sub::topic_callback, this, _1));

//     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//     "points2", 10);
//   }
// private:
//   void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "1");
//     // publisher_->publish(*msg);
//     std::cout<<"test\n";
//   }

  
//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<pub_and_sub>());
//   rclcpp::shutdown();
//   return 0;
// }

// class pub_and_sub : public rclcpp::Node
// {
//   public:
//     pub_and_sub() : Node("pub_and_sub_node")
//     {
//       publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/topic", 10);
//       subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 10, std::bind(&pub_and_sub::subscriber_part, this, _1));
//     }

//   private:
//     void subscriber_part(const sensor_msgs::msg::PointCloud2::ConstPtr& message) const
//     {
//       sensor_msgs::msg::PointCloud2 data = *message;
//       RCLCPP_INFO(this->get_logger(), "This is the subscriber working");
//       // publisher_->publish(data);
//       // publisher_part();
//     }

//     void publisher_part()
//     {
//       RCLCPP_INFO(this->get_logger(), "This is the publisher working");
//       // publisher_->publish(data);
//     }
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;    
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr subscriber_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<pub_and_sub>());
//   rclcpp::shutdown();
//   return 0;
// }