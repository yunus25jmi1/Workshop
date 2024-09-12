#!/usr/bin/env python3

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import Node class from rclpy
from std_msgs.msg import String  # Import standard message type String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')  # Initialize the node with a name
        self.publisher_ = self.create_publisher(String, 'string', 10)  # Create a publisher
        timer_period = 2.0  # Publish message every 2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Create a timer to trigger the callback

    def timer_callback(self):
        msg = String()  # Create a new String message
        msg.data = "Hi everyone, welcome to robotics workshop"  # Set the message data
        self.publisher_.publish(msg)  # Publish the message
        self.get_logger().info(f'Publishing: "{msg.data}"')  # Log the message

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python library
    node = TalkerNode()  # Create an instance of the TalkerNode
    rclpy.spin(node)  # Keep the node running and processing callbacks

    # Shutdown when finished
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
