#include <memory>       // Standard C++ header for smart pointers
#include <string>       // Standard C++ header for string manipulation
#include <chrono>       // Standard C++ header for time-related operations
#include <sstream>      // Standard C++ header for string stream operations

#include "rclcpp/rclcpp.hpp"            // ROS 2 C++ API header
#include "std_msgs/msg/string.hpp"      // ROS 2 standard message header for string data

// Define a class named SmallNodePublisher which inherits from rclcpp::Node
class SmallNodePublisher: public rclcpp::Node {

public:
  SmallNodePublisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;   // Shared pointer to a ROS 2 timer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;    // Shared pointer
                                                                     // to a ROS 2 publisher
  size_t counter_;    // Counter variable to keep track of messages published

private:
  void timer_callback_();   // Private method declaration for the timer callback function
};

// Constructor definition
SmallNodePublisher::SmallNodePublisher(): Node("small_node_publisher"), counter_(0) {   

  // Initialize the publisher with the node, specifying the topic name and queue size.
  // In this case we will use a hardcoded topic name, however this is a bad practice.
  publisher_ = this->create_publisher<std_msgs::msg::String>("small_node_topic", 10);

  // Create a timer with a callback function, setting the timer duration to 500 milliseconds.
  // This will result in timer callback being executed every 500 milliseconds.
  // What std::bind does is that it creates a function object, also known as a functor, that binds
  // the member function timer_callback_() to the specific instance of SmallNodePublisher (this).
  // This function object can then be called like a regular function, but it will invoke the member
  // function timer_callback_() of the specific instance.
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   std::bind(&SmallNodePublisher::timer_callback_, this));
}

// Definition of the timer callback function
void SmallNodePublisher::timer_callback_() {

  // Create a message object of type std_msgs::msg::String
  auto message = std_msgs::msg::String();

  // Create a stringstream object for constructing the message content
  std::stringstream str_stream;

  // Construct the message content with the current message counter value
  str_stream << "Message number " << counter_ << ".";
  // Increment the counter for the next message
  counter_++;

  // Assign the constructed string to the message data field.
  // Remember that method "str()" returns copy of stream contents so any
  // future modifications of stream will not affect message.data.
  message.data = str_stream.str();

  // Log a message indicating the content being published
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

  // Publish the message to the topic
  this->publisher_->publish(message);
}

// Main function
int main(int argc, char **argv) {

  // Initialize the ROS 2 runtime environment
  rclcpp::init(argc, argv);

  // Create a shared pointer to an instance of SmallNodePublisher
  auto node = std::make_shared<SmallNodePublisher>();

  // Spin the node, allowing it to process callbacks until shutdown
  rclcpp::spin(node);

  // Shutdown the ROS 2 runtime environment
  rclcpp::shutdown();

  // Return 0 to indicate successful execution
  return 0;
}
