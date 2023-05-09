#!/usr/bin/env python3

# importation des packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# création du node 
class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.conteur = 0

    def timer_callback(self):
        message = String()
        self.conteur += 1
        message.data = "Hello World! " + str(self.conteur)
        self.publisher_.publish(message)
        print(message.data)

def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher = HelloWorldPublisher()
    rclpy.spin(hello_world_publisher)
    hello_world_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

