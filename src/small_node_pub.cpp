#include <memory>
#include <string>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace chr = std::chrono_literals;


class SmallNodePublisher{

public:
  SmallNodePublisher()

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t counter_;

private:
  void timer_callback_();
};

SmallNodePublisher::SmallNodePublisher(): Node("small_node_publisher"), counter_(0){
  publisher_ = this->create_publisher("small_node_topic", 10);
  timer_ = this->create_wall_timer(chr::500ms, timer_callback_)
}

void SmallNodePublisher::timer_callback_(){
  auto message = std_msgs::msg::String();
  std:stringstream str_stream;
  str_stream << "Message number " << counter_ << ".";
  counter_++;
  message.data = str_stream.str();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  this->publisher_
}
