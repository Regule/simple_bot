// Include the header file for the ROS 2 C++ library
#include "rclcpp/rclcpp.hpp"

// Entry point of the program
int main(int argc, char **argv) {
    // Initialize the ROS 2 node with the provided command line arguments
    rclcpp::init(argc, argv);

    // Create a ROS 2 node with the name "tiny_node"
    auto node = rclcpp::Node::make_shared("tiny_node");

    // Spin the node, i.e., keep it running and responsive to events
    rclcpp::spin(node);

    // Shutdown the ROS 2 node and release any associated resources
    rclcpp::shutdown();

    // Return 0 to indicate successful execution of the program
    return 0;
}
