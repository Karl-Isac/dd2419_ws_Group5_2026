#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import sys          ## return to these
import math
import cv2
import numpy as np
from cv_bridge import CvBridge      # to convert between ros2 image and numpy array (for opencv)

def approx_to_polygon(contour):
    # Approximate contour to polygon
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
    return approx

def is_square(approx):
    # Check a bunch of conditions whether a contour is square-like
    # Approximation must have 4 corners
    if len(approx) != 4:
        return False

    # Must be convex
    if not cv2.isContourConvex(approx):
        return False

    # Area check
    area = cv2.contourArea(approx)                      
    min_area = 500                          # might need to finetune
    if area < min_area:
        return False

    # Check angles ~ 90 degrees using cosine
    pts = approx.reshape(4, 2)
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        p2 = pts[(i + 2) % 4]

        v1 = p0 - p1
        v2 = p2 - p1

        cos_angle = abs(
            np.dot(v1, v2) /
            (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
        )

        if cos_angle > 0.3:  # ~72–108 degrees                      # might need to finetune
            return False

    return True

def draw_cs_on_image(image,centerpoint,angle):
    # Draw cube center and orientation onto image
    arrow_length = 50
    theta = angle/180*math.pi
    cx,cy = centerpoint
    # X axis:
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx+arrow_length*math.cos(theta)),int(cy+arrow_length*math.sin(theta))),(0,0,255),2)
    # Y axis:
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx-arrow_length*math.sin(theta)),int(cy+arrow_length*math.cos(theta))),(0,255,0),2)


class Pickup(Node):
    def __init__(self):
        super().__init__('pickup')

        # Initialize the publisher                      # maybe disable to save even more computations
        self._pub = self.create_publisher(
            Image, '/arm/camera/image_debug', 10)
        self._pub2 = self.create_publisher(
            Image, '/arm/camera/image_debug2', 10)

        # Subscribe to the arm camera topic and call callback function on each received image
        self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10)
        
        self.cube_position_in_frame = []
        self.cube_orientation_in_frame = []
        
    def image_callback(self, msg: Image):
        # For each image received on /arm/camera/image_raw it updates the cube position and orientation variables
        # Known issues: it can detect multiple cubes/cube-like objects in the same frame, and both get written to the same attribute

        publish_debug_images = True

        # Convert ros2 Image to numpy array
        bridge = CvBridge()
        raw_image = bridge.imgmsg_to_cv2(       
            msg,
            desired_encoding='passthrough'
        )

        # Image preprocess - grayscale, blur so texture wont get detected as edges, Canny edge detection
        gray = cv2.cvtColor(raw_image, cv2.COLOR_YUV2GRAY_YUY2)
        gray = cv2.GaussianBlur(gray, (5, 5), 1.5)
        canny = cv2.Canny(gray, 50, 150)

        if publish_debug_images:
            bgr_image = cv2.cvtColor(raw_image,cv2.COLOR_YUV2BGR_YUY2)

        # Thicken edges so cube faces become distinctly separate
        kernel = np.ones((5,5), np.uint8)
        canny = cv2.dilate(canny, kernel)
        canny = cv2.bitwise_not(canny)

        contours, hierarchy = cv2.findContours(
            canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        for i in range(len(contours)):
            if hierarchy[0][i][2] == -1:            # look for only the innermost square, sometimes the cube shadow seems like an enveloping larger cube face
                poly = approx_to_polygon(contours[i])
                if is_square(poly):
                    rect = cv2.minAreaRect(poly)  # returns ((cx, cy), (width, height), angle)
                    centerpoint = rect[0]
                    angle = rect[2]         # in degrees
                    self.cube_position_in_frame = centerpoint
                    self.cube_orientation_in_frame = angle
                    if publish_debug_images:
                        cv2.drawContours(bgr_image, contours, i, (255,0,0), 4)
                        draw_cs_on_image(bgr_image,centerpoint,angle)

        if publish_debug_images:
            out_msg = bridge.cv2_to_imgmsg(         # convert the np array back to ros2 Image msg
                canny,
                encoding='mono8'#msg.encoding
            )
            out_msg.header = msg.header
            self._pub.publish(out_msg)      

            out_msg2 = bridge.cv2_to_imgmsg(         # convert the np array back to ros2 Image msg
                bgr_image,
                encoding='bgr8'
            )
            out_msg2.header = msg.header
            self._pub2.publish(out_msg2)          
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
