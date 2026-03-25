#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robp_interfaces.msg import ArmControl
from std_msgs.msg import String

import cv2
import numpy as np
from cv_bridge import CvBridge      # to convert between ros2 image and numpy array (for opencv)
import time

# Self written functions
from learning_tf2_py.arm_safe_republisher import jointmin,jointMAX
from learning_tf2_py.inverse_kin import inverse_kinematics_to_joint_states
from learning_tf2_py.pickup import saturate_difference,find_cube_in_image_msg


class Arm_control(Node):
    def __init__(self):
        super().__init__('pickup')

        # Initialize the publishers

        self._pub = self.create_publisher(              # pass these two to pickup function for debug
            Image, '/arm/camera/image_debug', 10)
        self._pub2 = self.create_publisher(
            Image, '/arm/camera/image_debug2', 10)
        self._pub3 = self.create_publisher(
            Image, '/arm/camera/image_debug3', 10)
        self._pub4 = self.create_publisher(
            Image, '/arm/camera/image_debug4', 10)
        

        # Subscribe to the arm camera topic and call callback function on each received image
        self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10)
        
        # Cube target within camera frame
        image_half_width = 320
        image_half_height = 240
        self.width_target = image_half_width
        self.height_target = image_half_height+200       # tunable, keep in mind that axis is flipped
        
        
    def image_callback(self, msg: Image):
            try:
                self.cube_position_in_frame, self.cube_orientation_in_frame = find_cube_in_image_msg(msg, self._pub, self._pub2, self._pub3, self._pub4, self.width_target, self.height_target, publish_debug_images=True)
            except Exception as ex:     # if crash is due to no cube detected then pass, otherwise reraise
                if ex.args[0] != "Cube not found in frame":
                    raise ex       

def main():
    rclpy.init()
    node = Arm_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
