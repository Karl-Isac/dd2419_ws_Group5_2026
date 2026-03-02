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

from std_msgs.msg import Float32  # debug

from learning_tf2_py.arm_safe_republisher import jointmin,jointMAX
from learning_tf2_py.inverse_kin import inverse_kinematics
    

class Arm_control(Node):
    def __init__(self):
        super().__init__('pickup')

        # Initialize the publishers

        self._pub = self.create_publisher(              # pass these two to pickup function for debug
            Image, '/arm/camera/image_debug', 10)
        self._pub2 = self.create_publisher(
            Image, '/arm/camera/image_debug2', 10)
        
        self._pub_control = self.create_publisher(
            ArmControl, '/arm/safe_control', 10)

        # Subscribe to the arm camera topic and call callback function on each received image
        self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10)
        
        

        
        self.cube_position_available = False

        self.joint1target = self.init_position[1]
        self.joint2target = self.init_position[2]
        self.joint3target = self.init_position[3]
        self.joint4target = self.init_position[4]
        self.joint5target = self.init_position[5]

        self.sideways_integral_term = 0

        # Send out a control action every 0.1 seconds
        timer_period = 0.1      # seconds               TODO change back
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def run(self):
        # State 0 - wait for pickup command:

        # State 1 - goto initial arm position
        # Initialize the arm position
        self.init_position = [10,120,50,150,100,120]            # [10,120,30,180,180,120] used during debug
        # Initial z, rho to move to rigth after
        self.z = 0.18
        self.rho = 0.175

        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [1000,1000,1000,1000,1000,1000]
        msg.position = self.init_position
        self._pub_control.publish(msg)
        time.sleep(1)
        # State 2 - goto z,rho where feedback control can be turned on, sub to camera

        # State 3 - feedback control ON, run until all errors are small

        # State 4 - feedback control OFF, goto lower z to pick up

        # State 5 - grip

        # State - goto initial position but gripper closed

        # State 7 - wait for place command

        # State 8 - Gripper release, goto initial (state1?) position 



    def timer_callback(self):
        if self.cube_position_available:
            try:
                # Control gains:
                k_sideways = 0.01#0.05                  commented values work with 0.5 sec timer
                k_sideways_integral = 0.01#0.1
                k_rotation = 1
                k_extension = 0.001

                # Previous targets:
                prev_joint1target = self.joint1target
                prev_joint2target = self.joint2target
                prev_joint3target = self.joint3target
                prev_joint4target = self.joint4target
                prev_joint5target = self.joint5target
                #prev_reach_target = self.reach_target

                

                cx,cy = self.cube_position_in_frame
                rotation = self.cube_orientation_in_frame
                self.cube_position_available = False
                # Sideways control (PI)
                sideways_error = self.width_target-cx
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

                # z-rho control (arm extend/contract + up-down, P)
                extension_error = self.height_target-cy
                self.rho = 0.175 + k_extension*extension_error
                # Saturate
                self.z = max(0.16, min(self.z, 0.175))
                self.rho = max(0.175, min(self.rho, 0.185))          # problem if value changes right after this line
                print("rho= "+str(self.rho))
                try:
                    result = inverse_kinematics(z=self.z,rho=self.rho)
                    alpha,beta = result
                    gamma = math.pi/2+alpha-beta            # arm camera pointing downwards constraint
                except:
                    self.get_logger().warn("Inverse kinematics failed for z={}, rho={}, target might be unreachable".format(self.z,self.rho))
                    # what TODO if no result

                # Translate alpha, beta, gamma into joint targets in the hardware's CS:
                self.get_logger().info('Alpha, beta, gamma: {}, {}, {}'.format(alpha*180/math.pi,beta*180/math.pi,gamma*180/math.pi))
                self.joint4target = 210-(alpha*180/math.pi)
                self.joint3target = 300-(beta*180/math.pi)
                self.joint2target = (gamma*180/math.pi)-60
                # Add hardcoded offset:
                self.joint2target = self.joint2target + 15



                # reach_error = self.height_target-cy         # height in image, forwards/backwards on the floor
                # self.reach_target = 20 + k_reach*reach_error

                # if self.reach_target > 50:          # saturate to avoid collisions
                #     self.reach_target = 50
                # elif self.reach_target < 20:
                #     self.reach_target = 20
                

                

                # Max motor speed: 60 deg / 0.22 sec (from github)
                # => 25 deg per 0.1 tick
                limit = 25
                # If the position target is too far from the previous one, lower it
                # Check whether all of these are reals, had some issues previously:
                assert all(isinstance(v, (int, float)) for v in (self.joint1target,prev_joint1target,self.joint5target,prev_joint5target,limit))
                self.joint1target = saturate_difference(self.joint1target,prev_joint1target,limit)
                self.joint2target = saturate_difference(self.joint2target,prev_joint2target,limit/10)
                self.joint3target = saturate_difference(self.joint3target,prev_joint3target,limit/10)
                self.joint4target = saturate_difference(self.joint4target,prev_joint4target,limit/10)
                self.joint5target = saturate_difference(self.joint5target,prev_joint5target,limit)
                #self.reach_target = saturate_difference(self.reach_target,prev_reach_target,limit/2)    # half the limit because reach translates to 2*reach degrees on one of the motors
                #self.reach_target = saturate_difference(self.reach_target,prev_reach_target,limit/100)    # half the limit because reach translates to 2*reach degrees on one of the motors
                #assert((self.reach_target >= 20) and (self.reach_target <= 50))
                
                self.get_logger().info('Joint 2,3,4 targets: '+ str(self.joint2target) +", "+ str(self.joint3target) +", "+ str(self.joint4target))

                
                
                msg = ArmControl()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.time = [100]*6  # move all joints in 100 ms (timer period), safe bcuz of saturation just above
                msg.position = [self.init_position[0],self.joint1target,self.joint2target,self.joint3target,self.joint4target,self.joint5target]

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

        # Define arm target in the image frame - its here due to debug reasons
        image_shape = gray.shape
        image_half_width = image_shape[1]/2
        image_half_height = image_shape[0]/2
        self.width_target = image_half_width
        self.height_target = image_half_height+100       # tunable, keep in mind that axis is flipped
        if publish_debug_images:
            draw_target_on_image(bgr_image,self.width_target,self.height_target)

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
    node = Arm_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
