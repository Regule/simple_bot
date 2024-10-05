#include <memory>       // Standard C++ header for smart pointers
#include <string>       // Standard C++ header for string manipulation
#include <chrono>       // Standard C++ header for time-related operations
#include <sstream>      // Standard C++ header for string stream operations

#include "rclcpp/rclcpp.hpp"            // ROS 2 C++ API header
#include "std_msgs/msg/string.hpp"      // ROS 2 standard message header for string data

// Define a class named ParametrizedPublisher which inherits from rclcpp::Node
class ParametrizedPublisher : public rclcpp::Node {

public:
  ParametrizedPublisher(); // Constructor declaration

private:
  rclcpp::TimerBase::SharedPtr timer_;   // Shared pointer to a ROS 2 timer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;    // Shared pointer 
                                                                     // to a ROS 2 publisher
  size_t counter_;    // Counter variable to keep track of messages published

private:
  void timer_callback_();   // Private method declaration for the timer callback function
};

// Constructor definition
ParametrizedPublisher::ParametrizedPublisher() : Node("parametrized_publisher"), counter_(0) {
  // Initialize a parameter descriptor object
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  // Set the description for the parameter descriptor
  param_desc.description = "Parameter description, this is optional";

  // Declare a parameter with name "demo_parameter", default value "default value",
  // and the parameter descriptor
  this->declare_parameter("demo_parameter", "default value", param_desc);

  // Initialize the publisher with the node, specifying the topic name and queue size
  // In this case, we use a hardcoded topic name, which is generally not recommended
  publisher_ = this->create_publisher<std_msgs::msg::String>("parametrized_topic", 10);

  // Create a timer with a callback function, setting the timer duration to 500 milliseconds
  // The timer callback will be executed every 500 milliseconds
  // std::bind creates a function object that binds the member function timer_callback_() 
  // to this instance of ParametrizedPublisher
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                   std::bind(&ParametrizedPublisher::timer_callback_, this));
}

// Definition of the timer callback function
void ParametrizedPublisher::timer_callback_() {
  // Retrieve the value of the parameter "demo_parameter" as a string
  std::string demo_param = this->get_parameter("demo_parameter").as_string();

  // Create a message object of type std_msgs::msg::String
  auto message = std_msgs::msg::String();

  // Create a stringstream object for constructing the message content
  std::stringstream str_stream;

  // Construct the message content with the current message counter value
  str_stream << "Message number " << counter_ << ", ";
  str_stream << "demo parameter value = " << demo_param << ".";

  // Increment the counter for the next message
  counter_++;

  // Assign the constructed string to the message data field
  // Note that str() returns a copy of the stream contents, so future modifications 
  // of the stream will not affect message.data
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

  // Create a shared pointer to an instance of ParametrizedPublisher
  auto node = std::make_shared<ParametrizedPublisher>();

  // Spin the node, allowing it to process callbacks until shutdown
  rclcpp::spin(node);

  // Shutdown the ROS 2 runtime environment
  rclcpp::shutdown();

  // Return 0 to indicate successful execution
  return 0;
}
