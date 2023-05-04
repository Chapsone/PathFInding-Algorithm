#!/usr/bin/env python3
# Library

import pygame, sys

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

from PIL import Image
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import threading


# couleur
UNKNOWN = (128, 128, 128)
FREE = (255, 255, 255)
OCCUPIED = (0, 0, 0)
PATH_COLOR = (0, 255, 0) # green
BLACK = (0 ,0 ,0)

# Dimension
DIM = 10

# setup pygame
HIDTH = 600
WIDTH = 1280
pygame.init()
WIN = pygame.display.set_mode((WIDTH, HIDTH))
pygame.display.set_caption("Pathfinder Algo")

def draw_path(self, surface):
    print(len(self.path))
    for i in range(len(self.path)):
        y , x = self.path[i]
        surface[x][y].fill(PATH_COLOR)
    return surface

def creat_surface(matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    surface = []
    matrix_grid = []
    for i in range(rows):
        surface.append([])
        matrix_grid.append([])
        for j in range(cols):
            surface[i].append(pygame.Surface((DIM,DIM)))
            matrix_grid[i].append(0)
            if matrix[i][j] == -1:
                surface[i][j].fill(UNKNOWN)
                matrix_grid[i][j] = 0
            elif matrix[i][j] == 0:
                matrix_grid[i][j] = 1
                surface[i][j].fill(FREE)
            else:
                matrix_grid[i][j] = 0
                surface[i][j].fill(OCCUPIED)
    return(surface, matrix_grid)

def draw(surface, matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        for j in range(cols):
            WIN.blit(surface[i][j],(j*DIM , i*DIM))

def make_grid(matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    for i in range(rows):
        pygame.draw.line(WIN, BLACK, (0, i*DIM), (DIM*cols, i*DIM))
        for j in range(cols):
            pygame.draw.line(WIN, BLACK, (j * DIM, 0), (j * DIM, DIM * rows))

def searching_path(start_pos, end_pos, matrix):
    # path finder
    # 1. creat a grid
    grid = Grid(matrix = matrix)

    # 2. creat a start and end cell
    start_x, start_y = start_pos
    end_x, end_y = end_pos

    start_node = grid.node(start_y, start_x)
    end_node = grid.node(end_y, end_x)
    # 3. creat a finder 
    finder = AStarFinder(diagonal_movement= DiagonalMovement.always)
    # 4. use a finder to find
    path, runs = finder.find_path(start_node, end_node, grid)
    return path

def get_pos(pos):
    mouse_pos = pos
    row = mouse_pos[1] // DIM
    col = mouse_pos[0] // DIM
    return(row,col)

class MapSubscriber(Node):

    def __init__(self):
        # Parameters
        self.start_pos = (0,0)
        self.end_pos = (0,0)
        self.start = None
        self.end = None
        self.path = []
        self.matrix = []
        self.clock = pygame.time.Clock()

        self.map_width = 0
        self.map_height = 0
        self.resolution = 1
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.robot_position_x = 0
        self.robot_position_y = 0


        #Node's definition
        rclpy.init(args=None)
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.subscription  # prevent unused variable warning
       
       # creat Thread
        self.thread = threading.Thread(target=self.background_task)
        self.thread.start()

    def odom_callback(self, msg):
        self.robot_position_x = msg.pose.pose.position.x
        self.robot_position_y = msg.pose.pose.position.y
        print("current robot posiont : ", self.robot_position_x, " ", self.robot_position_y)
        col = int((self.robot_position_x - self.map_origin_x) / self.resolution)
        row = int((self.robot_position_y - self.map_origin_y) / self.resolution)
        print(row, col)

    def map_callback(self, msg):
        map_array = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        # Transformation en une matrice 2D
        map_2d = [[0 for x in range(self.map_width)] for y in range(self.map_height)]
        for y in range(self.map_height):
            for x in range(self.map_width):
                map_2d[y][x] = map_array[y * self.map_width + x]
        self.matrix = map_2d

    def background_task(self):
        while True:
            print("i'm here")
            if self.matrix:
                surface, matrix_grid = creat_surface(self.matrix)
                draw(surface, self.matrix)
                make_grid(self.matrix)
                pygame.display.update()
                print(self.start, self.end)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        self.destroy_node()
                        rclpy.shutdown()
                        sys.exit()
                    # lef click on screen
                    if pygame.mouse.get_pressed()[0]:
                        pos = pygame.mouse.get_pos()
                        
                        if not self.start:
                            self.start_pos = get_pos(pos)
                            self.start = True
                            self.end = None
                            print(self.start_pos)

                        else:
                            self.end_pos = get_pos(pos)
                            self.end = True
                            print(self.end_pos)

                    if self.end:
                        self.path = searching_path(self.start_pos, self.end_pos, matrix_grid)
                        surface = draw_path(self, surface)
                        draw(surface, self.matrix)
                        make_grid(self.matrix)
                        pygame.display.update()
                        end = None
            self.clock.tick(60)

def main():
    node = MapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


