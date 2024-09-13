#!/usr/bin/env python3

# Importing necessary libraries from ROS2 Python client library (rclpy)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Define a class PyPublisher, inheriting from Node, to create a ROS2 node
class PyPublisher(Node):
    # The constructor of the class
    def __init__(self,
                 node_name:str = 'py_publisher',         # Default node name
                 topic_name:str = 'default_topic',       # Default topic name to publish messages
                 transmission_period:float = 0.5)-> None: # Time interval for publishing messages
        super().__init__(node_name)                       # Initialize the superclass (Node)
        # Create a publisher object for sending messages, queue size is 10
        self.publisher = self.create_publisher(String, topic_name, 10)
        # Create a timer object that calls the timer_callback method at regular intervals
        self.timer = self.create_timer(transmission_period, self.timer_callback)
        # Initialize a message count
        self.message_count:int = 0

    # The callback method for the timer
    def timer_callback(self):
        msg = String()                                     # Create a new String message object
        msg.data = f'Message number: {self.message_count}' # Assign a string to the message data
        self.publisher.publish(msg)                        # Publish the message to the topic
        # Log the message to the console
        self.get_logger().info(f'Publishing: {msg.data}')
        # Increment the message count
        self.message_count += 1

# Define the main function
def main(args=None):
    rclpy.init(args=args)           # Initialize the ROS2 Python client library
    py_pub = PyPublisher()          # Create an instance of the PyPublisher class
    rclpy.spin(py_pub)              # Keep the node alive to continue processing callbacks
    py_pub.destroy_node()           # Cleanly destroy the node instance
    rclpy.shutdown()                # Shutdown the ROS2 Python client library

# Entry point of the script
if __name__ == '__main__':
    main()
