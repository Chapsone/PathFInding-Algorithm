#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class ReadTopicMap(Node):
    def __init__(self):
        super().__init__('read__topic_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.callback,
            10)
        
    def callback(self, msg):
        self.map_array = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        print("La resolution est: ", self.resolution)
        print("L'origine (x,y) de la mappe est:(",
              self.map_origin_x,
              ",",
              self.map_origin_y,
              ")")
        print("Le Tableau est: ", self.map_array)


def main(args=None):
    rclpy.init(args=args)
    node = ReadTopicMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()