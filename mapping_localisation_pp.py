#!/usr/bin/env python3
# Library

import pygame, sys
import copy

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

from PIL import Image
import numpy as np
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import String



import threading


# couleur
UNKNOWN = (128, 128, 128)
FREE = (255, 255, 255)
OCCUPIED = (0, 0, 0)
PATH_COLOR = (0, 255, 0) # green
RED = (255,0,0)
BLACK = (0 ,0 ,0)

# Dimension
DIM = 6

# setup pygame
HIDTH = 600
WIDTH = 1280
pygame.init()
WIN = pygame.display.set_mode((WIDTH, HIDTH))
pygame.display.set_caption("Pathfinder Algo")

# loading robot image
image = pygame.image.load("boot.png")
robot = pygame.transform.scale(image, (image.get_width() // 2, image.get_height() // 2))

def degrees(angle_rad):
    angle_deg = angle_rad * int(180 / np.pi)
    return angle_deg

def display_robot(row, col, angle):
    rotated_robot = pygame.transform.rotate(robot, angle)
    cell_width = DIM  # Largeur de la cellule
    cell_height = DIM  # Hauteur de la cellule

    # Calculer les coordonnées pour centrer l'image dans la cellule
    x = col * cell_width + (cell_width - rotated_robot.get_width()) // 2
    y = row * cell_height + (cell_height - rotated_robot.get_height()) // 2

    WIN.blit(rotated_robot, (x, y))

def draw_path(self, surface):
    for i in range(len(self.path)):
        y , x = self.path[i]
        surface[x][y].fill(PATH_COLOR)
    return surface

def draw_path_reduit(self, surface):
    for i in range(len(self.path_reduit)):
        y , x = self.path_reduit[i]
        surface[x][y].fill(RED)
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
                matrix_grid[i][j] = 1
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

def get_angle(start_pos, end_pos):
    angle = 0
    start_x, start_y = start_pos
    end_x, end_y = end_pos
    dx = start_y - end_y
    dy = start_x - end_x

    if dx == 0 : dx = dx + 0.000001

    if (dx > 0 and dy >= 0): 
        angle = degrees(np.arctan(dy/dx))
    elif (dx < 0 and dy >= 0): 
        angle = 180 - abs(degrees(np.arctan(dy/dx)))
    elif (dx < 0 and dy <= 0): 
        angle = 180 + abs(degrees(np.arctan(dy/dx)))
    elif (dx > 0 and dy <= 0): 
        angle = 360 - abs(degrees(np.arctan(dy/dx)))

    return angle

def get_robot_oriantation_for_fuzzy(robot_oriantion):
    return robot_oriantion + 180

def get_angle_for_fuzzy(robot_oriantation_for_fuzzy, angle_bp):
    alpha = robot_oriantation_for_fuzzy - angle_bp
    if alpha > 180 :
        alpha = -(360 - alpha + angle_bp)
    return alpha

def reduction(path):
    gap = 3
    i = 0
    path_reduit = []
    while(i < len(path)):
        path_reduit.append(path[i])
        i += gap
    return path_reduit

def inverse_pos(path):
    return (path[1],path[0])

def augmented_matrix(matrix_grid):
    rows = len(matrix_grid)
    cols = len(matrix_grid[0])
    gap = 3
    augmented_matrix_grid = copy.deepcopy(matrix_grid)
    for i in range(gap, rows - gap):
        for j in range(gap, cols - gap):
            if matrix_grid[i][j] == 0:
                for a in range(-gap, gap + 1):
                    for b in range(-gap, gap + 1):
                        augmented_matrix_grid[i+a][j+b] = 0
                        
    return augmented_matrix_grid

class MapSubscriber(Node):

    def __init__(self):
        # Parameters
        self.start_pos = (0,0)
        self.end_pos = (0,0)
        self.start = None
        self.end = None
        self.path = []
        self.path_reduit = []
        self.matrix = []
        self.clock = pygame.time.Clock()

        self.map_width = 0
        self.map_height = 0
        self.resolution = 1
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.robot_position_x = 0
        self.robot_position_y = 0
        self.robot_pos = (0,0)
        self.row = 0
        self.col = 0
        self.angle = 0
        self.angle_bp = 0
        self.i = 0

        self.distance_for_fuzzy = Int32()
        self.angle_for_fuzzy = Float32()
        self.sync = String()

        #Node's definition
        rclpy.init(args=None)
        super().__init__('map_subscriber')

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.pub_distance = self.create_publisher(Int32, 'input_distance', 10)
        self.pub_angle = self.create_publisher(Float32, 'input_angle', 10)
        self.pub_sync = self.create_publisher(String, 'sync', 10)

        self.subscription  # prevent unused variable warning
       
       # creat Thread
        self.thread = threading.Thread(target=self.background_task)
        self.thread.start()

    def odom_callback(self, msg):
        self.robot_position_x = msg.pose.pose.position.x
        self.robot_position_y = msg.pose.pose.position.y
        self.col = int((self.robot_position_x - self.map_origin_x) / self.resolution)
        self.row = int((self.robot_position_y - self.map_origin_y) / self.resolution)

        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        self.angle = degrees(yaw)
        display_robot(self.row, self.col, 360 -(self.angle + 180))

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
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

    def send_input_for_fuzzy(self, robot_oriantation_for_fuzzy, angle_bp):
        dx = abs(self.robot_pos[1] - self.end_pos[1])
        dy = abs(self.robot_pos[0] - self.end_pos[0])

        self.distance_for_fuzzy.data = 0
        self.angle_for_fuzzy.data = 0.0
        if (np.sqrt(dx * dx + dy * dy)) : self.distance_for_fuzzy.data = 1
        print("distance between ", self.i, "and", self.i + 1, "is :", np.sqrt(dx * dx + dy * dy))
        print(self.robot_pos, self.end_pos)

        if not self.distance_for_fuzzy.data:
            if self.path_reduit :
                self.start_pos = inverse_pos(self.path_reduit[self.i])
                self.end_pos = inverse_pos(self.path_reduit[self.i + 1])
                if(self.i < len(self.path_reduit) - 2):
                    self.i+=1
                    self.sync.data = "start"
                else:
                    self.sync.data = "end"
                print(self.i, len(self.path_reduit))
                print("hello debug")

        self.angle_for_fuzzy.data = get_angle_for_fuzzy(robot_oriantation_for_fuzzy, angle_bp)
        self.pub_distance.publish(self.distance_for_fuzzy)
        self.pub_angle.publish(self.angle_for_fuzzy)
        self.pub_sync.publish(self.sync)
        print(self.distance_for_fuzzy.data, self.angle_for_fuzzy.data)

    def background_task(self):
        while True:
            if self.matrix:
                self.robot_pos = (self.row , self.col)
                self.angle_bp = get_angle(self.start_pos, self.end_pos)
                surface, matrix_grid = creat_surface(self.matrix)
                self.augmented_matrix_grid = augmented_matrix(matrix_grid)
                
                if self.path:
                    surface = draw_path(self, surface)
                    surface = draw_path_reduit(self, surface)
                draw(surface, self.matrix)
                make_grid(self.matrix)
                pygame.display.update()

                self.send_input_for_fuzzy(get_robot_oriantation_for_fuzzy(self.angle), self.angle_bp)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        self.destroy_node()
                        rclpy.shutdown()
                        sys.exit()

                    # lef click on screen
                    if pygame.mouse.get_pressed()[0]:
                        pos = pygame.mouse.get_pos()
                        self.end_pos = get_pos(pos)
                        self.i = 0
                        self.end = True

                    if self.end:
                        self.path = searching_path(self.robot_pos, self.end_pos, self.augmented_matrix_grid)
                        print(self.augmented_matrix_grid)
                        self.path_reduit = reduction(self.path)
                        self.end_pos = self.robot_pos
                        self.start_pos = self.robot_pos
                        self.end = None

            self.clock.tick(10)

def main():
    node = MapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()