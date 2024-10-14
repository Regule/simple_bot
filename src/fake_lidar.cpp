#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <cmath>
#include <utility>

//=================================================================================================
//                                HELPFULL DEFINES AND TYPEDEFS 
//=================================================================================================
#define MILLISECONDS_PER_SECOND 1000

//=================================================================================================
//                                       LIDAR CONFIG
//=================================================================================================
class LidarConfig
{
public:
    const char* PARAM_MIN_RANGE = "min_range";
    const char* PARAM_MAX_RANGE = "max_range";
    const char* PARAM_MIN_ANGLE = "min_angle";
    const char* PARAM_MAX_ANGLE = "max_angle";
    const char* PARAM_SAMPLE_COUNT = "sample_count";
    const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";

    const double DEFAULT_MIN_RANGE = 0.2;
    const double DEFAULT_MAX_RANGE = 10.0;
    const double DEFAULT_MIN_ANGLE = -M_PI;
    const double DEFAULT_MAX_ANGLE = M_PI;
    const int DEFAULT_SAMPLE_COUNT = 360;
    const double DEFAULT_SAMPLING_FREQUENCY = 10.0;

    const char* DESCRIPTION_MIN_RANGE = 
    "Smallest range that can be measured.";
    const char* DESCRIPTION_MAX_RANGE = 
    "Largest range that can be measured.";
    const char* DESCRIPTION_MIN_ANGLE=
    "Angle at which measurements start.";
    const char* DESCRIPTION_MIN_ANGLE=
    "Angle at which measurements end.";
    const char* DESCRIPTION_SAMPLE_COUNT = 
    "Number of measurements in single lidar sweep.";
    const char* DESCRIPTION_SAMPLING_FREQUENCY = 
    "Sampling frequency in Hz. Smallest sampling period is 1ms so there can be inacurracies.";

public:
    std::pair<double, double> range;
    std::pair<double, double> angle;
    int sample_count;
    double sampling_frequency;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    int get_sampling_period_ms() const;
    double get_angle_increment() const;
    double get_range_scope() const;
};


void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first  = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first  = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
}

int LidarConfig::get_sampling_period_ms() const
{
    return std::lround(MILLISECONDS_PER_SECOND/sample_frequency);
}

double LidarConfig::get_angle_increment() const
{
    return (angle.second - angle.first) / sample_count;
}

double LidarConfig::get_range_scope() const
{
    return range.first + (range.
}

//=================================================================================================
//                                       FAKE LIDAR 
//=================================================================================================
class FakeLidarNode : public rclcpp::Node
{
public:
    FakeLidarNode();

private:
    LidarConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_publisher;
    rclcpp::TimerBase::SharedPtr _lidar_timer;

private:

    void FakeLidar::_publish_lidar_scan();
    sensor_msgs::msg::LaserScan _prepare_lidar_message_with_metainfo() const;
    std::vector<float> _prepare_fake_scan() const;

};
    
sensor_msgs::msg::LaserScan FakeLidar::_prepare_lidar_message_with_metainfo() const
{
    auto message = sensor_msgs::msg::LaserScan();

    message.header.stamp = this->now();
    message.header.frame_id = "laser_frame";

    message.angle_min = _config.angle.first;
    message.angle_max = _config.angle.second;
    message.angle_increment = _config.get_angle_increment();
    message.time_increment = 0.0;
    message.scan_time = _config.get_scan_period_ms();
    message.range_min =  _config.range.first;
    message.range_max = _config.range.second;

    return message;
}
    
std::vector<float> FakeLidar::_prepare_fake_scan() const
{
    std::vector<float> ranges(num_readings_);
    for(int i = 0; i < num_readings_; ++i)
    {
      ranges[i] = _config.range.first + (_config.range.first - _config.range.second) *
            (std::sin(i * 0.1) + 1) / 2;
    }
    return ranges;
}

void FakeLidar::publish_lidar_scan()
{
    auto message = _prepare_lidar_message_with_metainfo();

    // Generate fake range data
    // Assign the ranges to the message
    message.ranges = ranges;

    // Publish the message
    lidar_publisher_->publish(message);
  }

FakeLidarNode::FakeLidarNode() : Node("fake_lidar_node")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _lidar_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    _lidar_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_sampling_period()),
        std::bind(&FakeLidarNode::publish_lidar_scan, this)
    );

}

//=================================================================================================
//                                      MAIN 
//=================================================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

