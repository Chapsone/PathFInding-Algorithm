#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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

# sets and membership of input
distance = ctrl.Antecedent(np.arange(0,2,1), 'distance')
distance['zero'] = fuzz.trimf(distance.universe, [0,0,0])
distance['one'] = fuzz.trimf(distance.universe, [1,1,1])

angle = ctrl.Antecedent(np.arange(-360, 360, 1), 'angle')
angle.automf(5)

# sets and membership of output
vl = 0.2
linear_vel = ctrl.Consequent(np.arange(0, vl + 0.5, vl), 'linear_velocity')
linear_vel['zero'] = fuzz.trimf(linear_vel.universe, [0,0,0])
linear_vel['one'] = fuzz.trimf(linear_vel.universe, [vl,vl,vl])

angular_vel = ctrl.Consequent(np.arange(-0.2, 0.2, 0.01), 'angular_velocity')
angular_vel.automf(5)

#fuzzy rules 
rule1 = ctrl.Rule(angle['poor'] & distance['zero'], [linear_vel['zero'], angular_vel['average']])
rule1.label = 'rule1'
rule2 = ctrl.Rule(angle['poor'] & distance['one'], [linear_vel['zero'], angular_vel['poor']])
rule2.label = 'rule2'

rule3 = ctrl.Rule(angle['mediocre'] & distance['zero'], [linear_vel['zero'], angular_vel['average']])
rule3.label = 'rule3'
rule4 = ctrl.Rule(angle['mediocre'] & distance['one'], [linear_vel['zero'], angular_vel['mediocre']])
rule4.label = 'rule4'

rule5 = ctrl.Rule(angle['decent'] & distance['zero'], [linear_vel['zero'], angular_vel['average']])
rule5.label = 'rule5'
rule6 = ctrl.Rule(angle['decent'] & distance['one'], [linear_vel['zero'], angular_vel['decent']])
rule6.label = 'rule6'

rule7 = ctrl.Rule(angle['good'] & distance['zero'], [linear_vel['zero'], angular_vel['average']])
rule7.label = 'rule7'
rule8 = ctrl.Rule(angle['good'] & distance['one'], [linear_vel['zero'], angular_vel['good']])
rule8.label = 'rule8'

rule9 = ctrl.Rule(angle['average'] & distance['zero'], [linear_vel['zero'], angular_vel['average']])
rule9.label = 'rule9'
rule10 = ctrl.Rule(angle['average'] & distance['zero'], [linear_vel['one'], angular_vel['average']])
rule10.label = 'rule10'

# Control system
control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10])

# Create control system simulation
simulation = ctrl.ControlSystemSimulation(control)

class HelloWorldSubscriber(Node):
    def __init__(self):
        self.input_angle = 0.0
        self.input_distance = 0
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
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

        self.cmd_pub_= self.create_publisher(Twist, "/cmd_vel",10)
        
        self.subscription  # prevent unused variable warning
       
       # creat Thread
        self.thread = threading.Thread(target=self.background_task)
        self.thread.start()

    
    def angle_callback(self, msg):
        self.input_angle = msg.data
        
    def distance_callback(self, msg):
        self.input_distance = msg.data
    
    def background_task(self):
        while True:

            #set input to fuzzy
            simulation.input['distance'] = self.input_distance 
            simulation.input['angle'] = self.input_angle 
            simulation.compute()
            
            self.msg.linear.x = simulation.output['linear_velocity']
            self.msg.angular.z = simulation.output['angular_velocity']

            print(self.msg.linear.x, self.msg.angular.z)
            self.cmd_pub_.publish(self.msg)
            self.clock.tick(60)

def main(args=None):
    rclpy.init(args=args)
    hello_world_subscriber = HelloWorldSubscriber()
    rclpy.spin(hello_world_subscriber)
    hello_world_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

