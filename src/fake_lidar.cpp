#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class FakeLidarNode : public rclcpp::Node
{
public:
  FakeLidarNode() : Node("fake_lidar_node")
  {
    // Create a publisher for the /scan topic
    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Timer to publish the LiDAR data at a regular interval
    timer_ = this->create_wall_timer(
        100ms, std::bind(&FakeLidarNode::publish_lidar_scan, this));

    // Initialize laser scan parameters
    range_min_ = 0.2;  // minimum range of the sensor
    range_max_ = 10.0; // maximum range of the sensor
    num_readings_ = 360; // Number of laser beams (1 degree per beam)
    angle_min_ = -M_PI;  // Start angle (in radians)
    angle_max_ = M_PI;   // End angle (in radians)
  }

private:
  void publish_lidar_scan()
  {
    auto message = sensor_msgs::msg::LaserScan();

    // Set the header with timestamp and frame
    message.header.stamp = this->now();
    message.header.frame_id = "laser_frame";

    // Set the laser scan parameters
    message.angle_min = angle_min_;
    message.angle_max = angle_max_;
    message.angle_increment = (angle_max_ - angle_min_) / num_readings_;
    message.time_increment = 0.0;
    message.scan_time = 1.0 / 10.0; // 10 Hz
    message.range_min = range_min_;
    message.range_max = range_max_;

    // Generate fake range data
    std::vector<float> ranges(num_readings_);
    for(int i = 0; i < num_readings_; ++i)
    {
      // Simulate ranges with a sinusoidal pattern
      ranges[i] = range_min_ + (range_max_ - range_min_) * (std::sin(i * 0.1) + 1) / 2;
    }

    // Assign the ranges to the message
    message.ranges = ranges;

    // Publish the message
    lidar_publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float range_min_;
  float range_max_;
  int num_readings_;
  float angle_min_;
  float angle_max_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

