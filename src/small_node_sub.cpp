#include <memory>       // Standard C++ header for smart pointers
#include <string>       // Standard C++ header for string manipulation
#include <chrono>       // Standard C++ header for time-related operations
#include <sstream>      // Standard C++ header for string stream operations

#include "rclcpp/rclcpp.hpp"            // ROS 2 C++ API header
#include "std_msgs/msg/string.hpp"      // ROS 2 standard message header for string data

using std::placeholders::_1;

class SmallNodeSubscriber: public rclcpp::Node {

public:
  SmallNodeSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

private:
  void subscription_callback_(std_msgs::msg::String::UniquePtr msg);
};

SmallNodeSubscriber::SmallNodeSubscriber(): Node("small_node_subscriber"){
  subscription_ = this->create_subscription<std_msgs::msg::String>("small_node_topic", 10,
                                                                   std::bind(&SmallNodeSubscriber::subscription_callback_, this, _1));
}

void SmallNodeSubscriber::subscription_callback_(std_msgs::msg::String::UniquePtr msg){
  RCLCPP_INFO(this->get_logger(), "Recieved message : '%s'", msg->data.c_str());
}

// Main function
int main(int argc, char **argv) {

  // Initialize the ROS 2 runtime environment
  rclcpp::init(argc, argv);

  // Create a shared pointer to an instance of SmallNodePublisher
  auto node = std::make_shared<SmallNodeSubscriber>();

  // Spin the node, allowing it to process callbacks until shutdown
  rclcpp::spin(node);

  // Shutdown the ROS 2 runtime environment
  rclcpp::shutdown();

  // Return 0 to indicate successful execution
  return 0;
}
