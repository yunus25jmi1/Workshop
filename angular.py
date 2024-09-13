
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriverNode(Node):

    def __init__(self):
        super().__init__('driving_custom_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.linear_vel = 0.0
        self.radius = 0.0

    def get_parameters(self):
        try:
            self.linear_vel = float(input("Enter the linear velocity: "))
            self.radius = float(input("Enter the radius: "))
            if self.radius <= 0:
                raise ValueError("Radius must be greater than 0.")
        except ValueError as e:
            self.get_logger().error(f"Error: {e}")
            self.get_logger().error("Using default values: linear velocity = 0.0, radius = 1.0")
            self.linear_vel = 0.0
            self.radius = 1.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.linear.y = 0.0
        msg.angular.z = self.linear_vel / self.radius if self.radius != 0 else 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode()
    driver_node.get_parameters()  # Get parameters from user input
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
