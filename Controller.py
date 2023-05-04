#!/usr/bin/env python3
import serial 
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import pygame

class controller(Node):

    def __init__(self):
        super().__init__("mannette")
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range (pygame.joystick.get_count())]
        pygame.init()

       

        self.cmd_pub_= self.create_publisher(Twist, "/cmd_vel",10)
        self.create_timer(0.5, self.callback(joysticks))

    def callback(self, joysticks):
        msg=Twist()
        print(joysticks)
        print("Karim")
        vx=0.0
        vz=0.0
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYHATMOTION:
                    fleche = pygame.joystick.Joystick(0).get_hat(0)
                    if fleche == (1,0):
                        if vz > -0.8:
                            vz= vz - 0.1
                    if fleche == (-1,0):
                        if vz < 0.8:
                            vz= vz + 0.1 

                if event.type == pygame.JOYBUTTONDOWN:

                    if pygame.joystick.Joystick(0).get_button(4):
                        vx= vx - 0.05
               
                    if pygame.joystick.Joystick(0).get_button(5):
                        vx= vx + 0.05

                    if pygame.joystick.Joystick(0).get_button(3):
                        vx=0.0

                    if pygame.joystick.Joystick(0).get_button(2):
                        vz=0.0
                        
                msg.linear.x= vx
                msg.angular.z= vz
                print(str(vx) + " " + str(vz))	
                self.cmd_pub_.publish(msg)
                time.sleep(0.01)



def main(args=None):
    rclpy.init(args=args)
    node= controller()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=='__main__':
    main() 
