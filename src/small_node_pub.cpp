#include <memory>
#include <string>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace chr = std::chrono;


class SmallNodePublisher: public rclcpp::Node{

public:
  SmallNodePublisher();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t counter_;

private:
  void timer_callback_();
};

SmallNodePublisher::SmallNodePublisher(): Node("small_node_publisher"), counter_(0){
  publisher_ = this->create_publisher<std_msgs::msg::String>("small_node_topic", 10);
  timer_ = this->create_wall_timer(chr::milliseconds(500), std::bind(&SmallNodePublisher::timer_callback_, this));
}

void SmallNodePublisher::timer_callback_(){
  auto message = std_msgs::msg::String();
  std::stringstream str_stream;
  str_stream << "Message number " << counter_ << ".";
  counter_++;
  message.data = str_stream.str();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  this->publisher_->publish(message);
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = SmallNodePublisher::make_shared("small_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
