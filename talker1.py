
import rclpy  # Import ROS2 Python library
from rclpy.node import Node  # Import Node module
from std_msgs.msg import String  # Import String message type

# Class definition for our ROS2 publisher node
class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')  # Initialize the Node with the name 'my_publisher'
        self.publisher_ = self.create_publisher(String, 'topic', 10)  # Create a publisher
        self.timer = self.create_timer(0.5, self.timer_callback)  # Create a timer to call the callback every 0.5 seconds

    def timer_callback(self):
        msg = String()  # Create a new String message
        msg.data = 'Hello, ROS2'  # Assign data to the message
        self.publisher_.publish(msg)  # Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data)  # Log an info message

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 Python communication
    my_publisher = MyPublisher()  # Create a MyPublisher object
    rclpy.spin(my_publisher)  # Keep the node alive to continue publishing

    my_publisher.destroy_node()  # Cleanup the node
    rclpy.shutdown()  # Shutdown ROS2 Python communication

if __name__ == '__main__':
    main()