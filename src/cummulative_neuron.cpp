#include <memory>       // Standard C++ header for smart pointers
#include <string>       // Standard C++ header for string manipulation
#include <chrono>       // Standard C++ header for time-related operations
#include <sstream>      // Standard C++ header for string stream operations
#include <algorithm>

#include "rclcpp/rclcpp.hpp"            // ROS 2 C++ API header
#include "std_msgs/msg/string.hpp"      // ROS 2 standard message header for string data

using std::placeholders::_1;

//==================================================================================================
class CummulativeNeuron{
public:
  CummulativeNeuron(std::vector<double> weights,
                    double rest_potential,
                    double dampener_strength);

  void dampen();
  void add_signal(size_t index,
                  double strength);
  double fire();
private:
  std::vector<double> weights_;
  std::vector<double> signal_;
  double rest_potential_;
  double dampener_strength_;
};

CummulativeNeuron::CummulativeNeuron(std::vector<double> weights,
                                     double rest_potential,
                                     double dampener_strength):
                                     weights_(weights), 
                                     rest_potential_(rest_potential),
                                     dampener_strength_(dampener_strength)
{
}

CummulativeNeuron::dampen()
{
  for(int idx=0; idx<weights_.length(); idx++)
  {
    signal_[idx] = std::max(0, signal_[x]-dampener_strength_);
  }
}

//==================================================================================================
struct CummulativeNeuronParameters{
  std::vector<std::string> inupt_topics_;
  std::vector<double> weights_;
  std::string output_topic_;
};

class CummulativeNeuronNode: public rclcpp::Node {
public:
  CummulativeNeuronNode();
private:
  void load_parameters_();
private:
  CummulativeNeuronParameters parameters;
};

//==================================================================================================
// Main function
int main(int argc, char **argv) {
  // Initialize the ROS 2 runtime environment
  rclcpp::init(argc, argv);

  // Create a shared pointer to an instance of CummulativeNeuron 
  auto node = std::make_shared<CummulativeNeuronNode>();

  // Spin the node, allowing it to process callbacks until shutdown
  rclcpp::spin(node);

  // Shutdown the ROS 2 runtime environment
  rclcpp::shutdown();

  // Return 0 to indicate successful execution
  return 0;
}
