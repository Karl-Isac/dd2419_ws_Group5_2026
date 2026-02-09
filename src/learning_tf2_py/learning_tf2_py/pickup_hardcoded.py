import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty
from robp_interfaces.msg import ArmControl
import time

class PickupHardcoded(Node):
    def __init__(self):
        super().__init__('pickup_hardcoded')
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        
    
    def run(self):
        time.sleep(3)           # a delay to make recording easier
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [3000,3000,3000,3000,3000,3000]
        msg.position = [10,120,60,170,60,120]
        self.pub.publish(msg)
        time.sleep(3)
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [3000,3000,3000,3000,3000,3000]
        msg.position = [105,120,60,170,60,120]
        self.pub.publish(msg)
        time.sleep(3)
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [3000,3000,3000,3000,3000,3000]
        msg.position = [105,120,60,120,120,120]
        self.pub.publish(msg)
        time.sleep(5)
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [3000,3000,3000,3000,3000,3000]
        msg.position = [10,120,60,120,120,120]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PickupHardcoded()
    try:
        node.run()
    except KeyboardInterrupt as kx:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
