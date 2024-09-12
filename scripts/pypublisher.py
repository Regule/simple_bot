#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PyPublisher(Node):

    def __init__(self,
                 node_name:str = 'py_publisher',
                 topic_name:str = 'default_topic',
                 transmission_period:float = 0.5)-> None:
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, topic_name, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(transmission_period, self.timer_callback)
        self.message_count:int = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Mesage number: {self.message_count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.message_count += 1


def main(args=None):
    rclpy.init(args=args)
    py_pub = PyPublisher()
    rclpy.spin(py_pub)
    py_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

