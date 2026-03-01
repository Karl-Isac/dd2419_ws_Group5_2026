#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import ArmControl
import time

import random

class Pickup(Node):
    def __init__(self):
        super().__init__('pickup')

        frequency = 10
        self._pub_control = self.create_publisher(
            ArmControl, '/arm/safe_control', frequency)
        
        # Initialize the arm position
        self.init_position = [10,120,20,190,120,120]            # [10,120,30,180,180,120] used during debug
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [1000,1000,1000,1000,1000,1000]
        msg.position = self.init_position
        self._pub_control.publish(msg)
        time.sleep(1)

        timer_period = 1/frequency      # seconds  
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        try:
            msg = ArmControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.time = [1000]*6 
            noise = random.uniform(-0.1,0.1)        # add on 0.1 degree noise
            #################################################################################
            # Test all of these with varying timer period and publishing rate too
            # Test without safe control wrapper too
            # Maybe vary the msg.time too
            #################################################################################
            # Test 1 - no noise, same command all the time:
            msg.position = [self.init_position[0],self.init_position[1],self.init_position[2],self.init_position[3],self.init_position[4],self.init_position[5]]
            # Test 2 - add noise to base rotation:
            msg.position = [self.init_position[0],self.init_position[1],self.init_position[2],self.init_position[3],self.init_position[4],self.init_position[5]+noise]
            # Test 3 - add noise to gripper rotation:
            msg.position = [self.init_position[0],self.init_position[1]+noise,self.init_position[2],self.init_position[3],self.init_position[4],self.init_position[5]]
            # Test 4 - add noise to one of the contributing links
            msg.position = [self.init_position[0],self.init_position[1],self.init_position[2],self.init_position[3],self.init_position[4]+noise,self.init_position[5]]
            
            print(msg.position)
            self._pub_control.publish(msg)

        except Exception as e:
            # If something breaks in the controller, goto initial position, assumed to be safe:
            self.get_logger().fatal("Error in controller code, moving to safe state and shutting down")
            msg = ArmControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.time = [3000,3000,3000,3000,3000,3000]
            msg.position = self.init_position
            self._pub_control.publish(msg)
            raise
        return
        
        

def main():
    rclpy.init()
    node = Pickup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
