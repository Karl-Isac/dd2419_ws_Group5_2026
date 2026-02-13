import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty
from robp_interfaces.msg import ArmControl
import time

class ArmSafeRepublisher(Node):
    def __init__(self):
        super().__init__('arm_safe_republisher')
        self.pub = self.create_publisher(ArmControl, '/arm/control', 10)

        self.create_subscription(
            ArmControl, '/arm/safe_control', self.control_request_callback, 10)

    def control_request_callback(self, msg: ArmControl):
        # If any unsafe control action is received it either:
        # - saturates the control action
        # - if the above is not possible it doesnt publish the control action

        # Separate joint limits:
        joints = msg.position
        jointmin = [0,0,15,10,40,0]               # min and max allowed angles, 0 - gripper, 5 - rotating base
        jointMAX = [105,240,230,230,200,160]
        for i in range(6):
            if joints[i] < jointmin[i]:
                joints[i] = jointmin[i]
            elif joints[i] > jointMAX[i]:
                joints[i] = jointMAX[i]

        msg.position = joints
        self.pub.publish(msg)

        # time.sleep(3)           # a delay to make recording easier
        # msg = ArmControl()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.time = [3000,3000,3000,3000,3000,3000]
        # msg.position = [10,120,60,170,60,120]
        # self.pub.publish(msg)
        # time.sleep(3)
        # msg = ArmControl()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.time = [3000,3000,3000,3000,3000,3000]
        # msg.position = [105,120,60,170,60,120]
        # self.pub.publish(msg)
        # time.sleep(3)
        # msg = ArmControl()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.time = [3000,3000,3000,3000,3000,3000]
        # msg.position = [105,120,60,120,120,120]
        # self.pub.publish(msg)
        # time.sleep(5)
        # msg = ArmControl()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.time = [3000,3000,3000,3000,3000,3000]
        # msg.position = [10,120,60,120,120,120]
        # self.pub.publish(msg)

def main():
    rclpy.init()
    node = ArmSafeRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
