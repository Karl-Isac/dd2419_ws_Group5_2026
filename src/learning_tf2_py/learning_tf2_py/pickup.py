#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robp_interfaces.msg import ArmControl

import sys          ## return to these
import math
import cv2
import numpy as np
from cv_bridge import CvBridge      # to convert between ros2 image and numpy array (for opencv)
import time

from learning_tf2_py.arm_safe_republisher import jointmin,jointMAX

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
        
    # Side length check (Is it a square or a rectangle)
    side_lengths = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        side_length = np.linalg.norm(p1 - p2)
        side_lengths.append(side_length)
    min_side = min(side_lengths)
    max_side = max(side_lengths)

    aspect_tolerance = 0.2   # 20% tolerance, tunable parameter
    if (max_side - min_side) / max_side > aspect_tolerance:
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

def saturate_difference(current,previous,limit):
    print(previous + limit)
    print(previous - limit)
    if abs(current - previous) > limit:
        if (current - previous) > 0:
            return previous + limit
        else:
            return previous - limit
    else:
        return current

        


class Pickup(Node):
    def __init__(self):
        super().__init__('pickup')

        # Initialize the publisher                      # maybe disable to save even more computations
        self._pub = self.create_publisher(
            Image, '/arm/camera/image_debug', 10)
        self._pub2 = self.create_publisher(
            Image, '/arm/camera/image_debug2', 10)
        
        self._pub_control = self.create_publisher(
            ArmControl, '/arm/safe_control', 10)

        # Subscribe to the arm camera topic and call callback function on each received image
        self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10)
        
        # Initialize the arm position
        self.init_position = [10,120,30,180,100,120]            # [10,120,30,180,180,120] used during debug
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [1000,1000,1000,1000,1000,1000]
        msg.position = self.init_position
        self._pub_control.publish(msg)
        time.sleep(1)

        
        self.cube_position_available = False

        self.joint5target = 120
        self.joint1target = 120

        self.sideways_integral_term = 0

        # Send out a control action every 0.1 seconds
        timer_period = 0.1   #debug 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        if self.cube_position_available:
            try:
                start = time.perf_counter()
                # Control gains:
                k_sideways = 0.01#0.05                  commented values work with 0.5 sec timer
                k_sideways_integral = 0.01#0.1
                k_rotation = 1

                # Previous targets:
                prev_joint1target = self.joint1target
                prev_joint5target = self.joint5target

                image_half_width = self.image_shape[1]/2
                cx,cy = self.cube_position_in_frame
                rotation = self.cube_orientation_in_frame
                self.cube_position_available = False
                # Sideways control (PI)
                sideways_error = image_half_width-cx
                self.sideways_integral_term = self.sideways_integral_term + k_sideways_integral*sideways_error
                self.joint5target = 120 + k_sideways*sideways_error + self.sideways_integral_term
                # Anti integral windup -> simple clamping
                k_sideways_anti_windup = 100 * k_sideways_integral        # tunable
                if self.joint5target > jointMAX[5]:
                    sat_amount = self.joint5target - jointMAX[5]
                    self.sideways_integral_term = self.sideways_integral_term - sat_amount * k_sideways_anti_windup
                    self.joint5target = jointMAX[5]
                elif self.joint5target < jointmin[5]:
                    self.joint5target = jointmin[5]
                
                # Rotation control (P)
                rotation_error = rotation % 90
                if rotation_error > 45:
                    rotation_error = rotation_error - 90
                self.joint1target = 120 + k_rotation*rotation_error

                # Max motor speed: 60 deg / 0.22 sec (from github)
                # => 25 deg per 0.1 tick
                limit = 25
                # If the position target is too far from the previous one, lower it
                # Check whether all of these are reals, had some issues previously:
                assert all(isinstance(v, (int, float)) for v in (self.joint1target,prev_joint1target,self.joint5target,prev_joint5target,limit))
                self.joint1target = saturate_difference(self.joint1target,prev_joint1target,limit)
                self.joint5target = saturate_difference(self.joint5target,prev_joint5target,limit)
                self.get_logger().info("Previous target: " + str(prev_joint5target))
                self.get_logger().info("Current target: " + str(self.joint5target))
                self.get_logger().info("Integral term: " + str(self.sideways_integral_term)+"\n")

                

                msg = ArmControl()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.time = [100]*6  # move all joints in 100 ms (timer period), safe bcuz of saturation just above
                msg.position = [self.init_position[0],self.joint1target,self.init_position[2],self.init_position[3],self.init_position[4],self.joint5target]

                self._pub_control.publish(msg)
                end = time.perf_counter()
                print(f"Execution time: {end - start:.6f} seconds")

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
        self.image_shape = gray.shape

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
                    self.cube_position_available = True
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
