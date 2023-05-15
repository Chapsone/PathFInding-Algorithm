#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import pygame, sys
import threading

# Color
UNKNOWN = (128, 128, 128)
FREE = (255, 255, 255)
OCCUPIED = (0, 0, 0)
BLACK = (0 ,0 ,0)
BARRER = (0, 255, 0) # green

# Dimension
DIM = 10

# setup pygame
HIDTH = 600
WIDTH = 1280
pygame.init()
SCREEN = pygame.display.set_mode((WIDTH, HIDTH))
pygame.display.set_caption("Pdraw_map_vr2")

def creat_surface(matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    surface = []
    for i in range(rows):
        surface.append([])
        for j in range(cols):
            surface[i].append(pygame.Surface((DIM,DIM)))
            if matrix[i][j] == -1:
                surface[i][j].fill(UNKNOWN)
            elif matrix[i][j] == 0:
                surface[i][j].fill(FREE)
            else:
                surface[i][j].fill(OCCUPIED)
    return(surface)

# the function that display the surface in the Screen
def draw(surface, matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        for j in range(cols):
            SCREEN.blit(surface[i][j],(j*DIM , i*DIM))

# that one is dilplaying the limit of any cell so that we can see all of theme
def make_grid(matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        pygame.draw.line(SCREEN, BLACK, (0, i*DIM), (DIM*cols, i*DIM))
        for j in range(cols):
            pygame.draw.line(SCREEN, BLACK, (j * DIM, 0), (j * DIM, DIM * rows))

class ReadTopicMap(Node):
    def __init__(self):

        # parametre 
        self.matrix = []
        self.clock = pygame.time.Clock()
        self.map_width = 0
        self.map_height = 0
        self.resolution = 1
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.row = 0
        self.col = 0

        # ROS2
        super().__init__('read__topic_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.callback,
            10)
        
        # creat Thread
        self.thread = threading.Thread(target=self.main_class)
        self.thread.start()
        
        
    def callback(self, msg):
        self.map_array = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
         # convert 1D array into 2D
        map_2d = [[0 for x in range(self.map_width)] for y in range(self.map_height)]
        for y in range(self.map_height):
            for x in range(self.map_width):
                map_2d[y][x] = self.map_array[y * self.map_width + x]
        self.matrix = map_2d
    
    def main_class(self):
        while True:
            if self.matrix:
                surface = creat_surface(self.matrix)
                draw(surface, self.matrix)
                make_grid(self.matrix)
                pygame.display.update()

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        self.destroy_node()
                        rclpy.shutdown()
                        sys.exit()

            self.clock.tick(60)


def main(args=None):
    rclpy.init(args=args)
    node = ReadTopicMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()