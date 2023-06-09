#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import pygame, sys
pygame.init()

import threading

# FLC: Fuzzy Logic Controller
import numpy as np 
import skfuzzy as fuzz
from skfuzzy import control as ctrl

#sets input and output
angle = ctrl.Antecedent(np.linspace(-180, 180, 360), 'angle')
angular_vel = ctrl.Consequent(np.linspace(-0.6, 0.6, 360), 'angular_vel')

# Define the parameters for each membership function
angle['poor'] = fuzz.trapmf(angle.universe, [-180, -180, -80, -30])
angle['mediocre'] = fuzz.trimf(angle.universe, [-50, -25, 0])

angle['average'] = fuzz.trimf(angle.universe, [-5, 0, 5])

angle['decent'] = fuzz.trimf(angle.universe, [0, 25, 50])
angle['good'] = fuzz.trapmf(angle.universe, [30, 80, 180, 180])

#output
angular_vel['poor'] = fuzz.trapmf(angular_vel.universe, [-0.6, -0.6, -0.4, -0.2])
angular_vel['mediocre'] = fuzz.trimf(angular_vel.universe, [-0.3, -0.15, 0])

angular_vel['average'] = fuzz.trimf(angular_vel.universe, [-0.1, 0, 0.1])

angular_vel['decent'] = fuzz.trimf(angular_vel.universe, [0, 0.15, 0.3])
angular_vel['good'] = fuzz.trapmf(angular_vel.universe, [0.2, 0.4, 0.6, 0.6])

#fuzzy rules 
rule2 = ctrl.Rule(angle['poor'], angular_vel['good'])
rule2.label = 'rule2'

rule4 = ctrl.Rule(angle['mediocre'], angular_vel['decent'])
rule4.label = 'rule4'

rule6 = ctrl.Rule(angle['decent'], angular_vel['mediocre'])
rule6.label = 'rule6'

rule8 = ctrl.Rule(angle['good'], angular_vel['poor'])
rule8.label = 'rule8'

rule10 = ctrl.Rule(angle['average'], angular_vel['average'])
rule10.label = 'rule10'

# Control system
control = ctrl.ControlSystem([rule2, rule4, rule6, rule8, rule10])

# Create control system simulation
simulation = ctrl.ControlSystemSimulation(control)

class HelloWorldSubscriber(Node):
    def __init__(self):
        self.input_angle = 0.0
        self.input_distance = 0
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.sync = "end"
        self.clock = pygame.time.Clock()

        super().__init__('fuzzy_node')

        self.subscription = self.create_subscription(
            Int32,
            'input_distance',
            self.distance_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float32,
            'input_angle',
            self.angle_callback,
            10)
        
        self.subscription = self.create_subscription(
            String,
            'sync',
            self.sync_callback,
            10)

        self.cmd_pub_= self.create_publisher(Twist, "/cmd_vel",10)
        
        self.subscription  # prevent unused variable warning
       
       # creat Thread
        self.thread = threading.Thread(target=self.background_task)
        self.thread.start()

    def sync_callback(self, msg):
        self.sync = msg.data
    
    def angle_callback(self, msg):
        self.input_angle = msg.data
        
    def distance_callback(self, msg):
        self.input_distance = msg.data
    
    def background_task(self):
        while True:
            if self.sync == "start" :
                #set input to fuzzy
                simulation.input['angle'] = self.input_angle 
                simulation.compute()

                # output
                self.msg.angular.z = simulation.output['angular_vel']

                # classical commande for the linear velocity 
                if(self.input_distance and int(self.input_angle) == 0): self.msg.linear.x = 0.08
                else: self.msg.linear.x = 0.0
                #self.msg.linear.x = 0.1
                print(self.input_angle, self.msg.linear.x, self.msg.angular.z)
                self.cmd_pub_.publish(self.msg)
                self.clock.tick(10)

def main(args=None):
    rclpy.init(args=args)
    hello_world_subscriber = HelloWorldSubscriber()
    rclpy.spin(hello_world_subscriber)
    hello_world_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

