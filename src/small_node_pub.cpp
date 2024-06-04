#include <memory>
#include <string>
#include <chrono>

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
  void timer_callback();
};
