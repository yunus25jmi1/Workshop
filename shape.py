#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
import time

PI = 3.1415926535897

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_shape_node')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_msg = Twist()

    def move_forward(self, target_distance):
        self.get_logger().info("Moving the turtle forward")
        speed = 2.0
        self.vel_msg.linear.x = speed
        start_time = time.time()  # Record the current time
        current_distance = 0.0

        while current_distance < target_distance:
            self.velocity_publisher.publish(self.vel_msg)
            current_distance = speed * (time.time() - start_time)  # Calculate the distance based on time

        # Stop the turtle
        self.vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(self.vel_msg)

    def rotate(self, angle_degrees):
        self.get_logger().info("Rotating the turtle")
        speed = 50.0
        angular_speed = speed * 2 * PI / 360
        print("angular_speed",angular_speed)
        relative_angle = angle_degrees * 2 * PI / 360
        self.vel_msg.angular.z = angular_speed

        start_time = time.time()  # Record the current time
        current_angle = 0.0

        while current_angle < relative_angle:
            self.velocity_publisher.publish(self.vel_msg)
            current_angle = angular_speed * (time.time() - start_time)  # Calculate the angle based on time

        # Stop the turtle
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)

    def make_square(self):
        for _ in range(4):
            self.move_forward(2)
            time.sleep(0.1)
            self.rotate(90)

    def make_rectangle(self):
        time.sleep(0.5)
        self.move_forward(4)
        time.sleep(0.5)
        self.rotate(90)
        self.move_forward(2)
        time.sleep(0.5)
        self.rotate(90)
        time.sleep(0.5)
        self.move_forward(4)
        time.sleep(0.5)
        self.rotate(90)
        time.sleep(0.5)
        self.move_forward(2)
        time.sleep(0.5)
        self.rotate(90)
        time.sleep(0.5)

    def make_star(self):
        angle = 144
        for _ in range(5):
            self.move_forward(2)
            self.rotate(angle)
            self.move_forward(2)
            self.rotate(72 - angle)

    def reset_turtle(self):
        client = self.create_client(Empty, '/reset')
        client.wait_for_service()
        client.call_async(Empty.Request())

    def spawn_bot(self):
        self.get_logger().info("Spawning a new turtle")
        client = self.create_client(Spawn, '/spawn')
        client.wait_for_service()
        spawn_request = Spawn.Request()

        spawn_request.x = float(input("Enter the x coordinate: "))
        spawn_request.y = float(input("Enter the y coordinate: "))
        spawn_request.theta = float(input("Enter the theta in radians: "))
        spawn_request.name = "my_turtle"

        client.call_async(spawn_request)


def main():
    rclpy.init()
    turtle_controller = TurtleController()

    while rclpy.ok():
        shape = input("Enter the shape (square/rectangle/star/spawn): ")

        if shape == "square":
            turtle_controller.make_square()
            time.sleep(3)
            turtle_controller.reset_turtle()
        elif shape == "rectangle":
            turtle_controller.make_rectangle()
            time.sleep(3)
            turtle_controller.reset_turtle()
        elif shape == "star":
            turtle_controller.make_star()
            time.sleep(3)
            turtle_controller.reset_turtle()
        elif shape == "spawn":
            turtle_controller.spawn_bot()
        else:
            print("Invalid shape, please enter again.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
