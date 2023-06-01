#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial 
import time


ser = serial.Serial('/dev/ttyACM0', 115200, timeout= 1.0)
time.sleep(0.5)
ser.reset_input_buffer()

class controller(Node):
    def __init__(self):

        super().__init__("Slave")

        self.cmd_listnerr_ =self.create_subscription(Twist, "/demo/cmd_vel", self.data_usage,10)

    def data_usage(self, msg : Twist):

        vx = msg.linear.x
        vz = msg.angular.z

        cmd = str(vx) + 'a' + str(vz) + 'e'
        ser.write(cmd.encode('utf-8'))
        ser.write("\n".encode('utf-8'))
    
        print(cmd)
        time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node= controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main() 
