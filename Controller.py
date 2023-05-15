#!/usr/bin/env python3
import serial 
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import pygame

import threading


class controller(Node):

    def __init__(self):
        # Parameters
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x)
                      for x in range
                        (pygame.joystick.get_count())]
        pygame.init()

        super().__init__("mannette")
        self.cmd_pub_= self.create_publisher(Twist, "/cmd_vel",10)
        # self.create_timer(0.5, self.callback)

        # creat Thread
        self.thread = threading.Thread(target=self.main_class(joysticks))
        self.thread.start()

    def callback(self):
        print(str(self.msg.linear.x) + " " + str(self.msg.angular.z))	
        self.cmd_pub_.publish(self.msg)

    def main_class(self, joysticks):
        vx = 0.0
        vz = 0.0
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
                        
                self.msg.linear.x = vx
                self.msg.angular.z = vz
                print(str(self.msg.linear.x) + " "
                       + str(self.msg.angular.z))	
                self.cmd_pub_.publish(self.msg)
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node= controller()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=='__main__':
    main() 