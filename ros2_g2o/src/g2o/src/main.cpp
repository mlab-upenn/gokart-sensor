#include <chrono>
#include <memory>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>    
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// To test this code run every command in its own sourced terminal:
// ros2 topic pub /temp_1 std_msgs/msg/String "data: Hello World 1!"
// ros2 topic pub /temp_2 std_msgs/msg/String "data: Hello World 2!"
// ros2 topic echo /g2o
// source in g2o folder ". install/setup.bash" run "ros2 run ros2_g2o ros2_g2o --ros-args --log-level debug"
// remove "--ros-args --log-level debug" to not see the debug outputs

// Define message type for both Subscribers and Publisher
using pub1_topic = std_msgs::msg::String;
using sub1_topic = std_msgs::msg::String;
using sub2_topic = std_msgs::msg::String;


class g2oNode : public rclcpp::Node {
 public:
  g2oNode() : Node("g2o") {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    pub1 = this->create_publisher<pub1_topic>("/g2o", qos);

    sub1.subscribe(this, "temp_1", rmw_qos_profile);
    sub2.subscribe(this, "temp_2", rmw_qos_profile);

    temp_sync_ = std::make_shared<message_filters::TimeSynchronizer<sub1_topic, sub2_topic>>(sub1, sub2, 10);
    temp_sync_->registerCallback(std::bind(&g2oNode::TempSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

 private:

  void TempSyncCallback(const sub1_topic::ConstSharedPtr& sub1_msg, const sub2_topic::ConstSharedPtr& sub2_msg)
  {
    // RCLPP_INFO behaves as printf and therefore can't print strings directly --> use c_str() that creates pointer to the string
    RCLCPP_INFO(this->get_logger(),"I heard and synchronized the following data: %s, %s",sub1_msg->data.c_str(), sub2_msg->data.c_str());
    pub1->publish(*sub1_msg);
  }

  rclcpp::Publisher<pub1_topic>::SharedPtr pub1;
  message_filters::Subscriber<sub1_topic> sub1;
  message_filters::Subscriber<sub2_topic> sub2;
  std::shared_ptr<message_filters::TimeSynchronizer<sub1_topic, sub2_topic>> temp_sync_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g2oNode>());
  rclcpp::shutdown();
  return 0;
}
