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
from learning_tf2_py.pickup import saturate_difference,find_cube_in_image_msg, is_the_target_cube_colored


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
            Image, '/arm/camera/image_debug3', 10)
        self._pub5 = self.create_publisher(
            Image, '/arm/camera/image_debug_gripper_crop', 10)
        
        self._pub_control = self.create_publisher(
            ArmControl, '/arm/safe_control', 1)

        # Subscribe to the arm camera topic and call callback function on each received image
        self.create_subscription(
            Image, '/arm/camera/image_raw', self.image_callback, 10)
        
        # Listen for commands:
        self.create_subscription(
            String,
            "/arm/cmd",
            self.cmd_callback,
            10
        )

        self.report_publisher = self.create_publisher(
            String,
            "/arm/report_back",
            10
        )

        self.wait_for_pickup_command = False
        self.wait_for_place_command = False
        self.visual_servoing_ON = False
        self.look_at_gripper_contents = False
        
        self.init_position = [10,120,50,150,100,120]
        self.joint0grip_value = 105

        # Cube target within camera frame
        image_half_width = 320
        image_half_height = 240
        self.width_target = image_half_width
        self.height_target = image_half_height+200       # tunable, keep in mind that axis is flipped

        # When visual servoing send out a control action every .1 sec
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_callback(self, msg):
        content = msg.data
        if self.wait_for_pickup_command:
            if content[:4] == "pick":
                self.get_logger().info("Proceeding to pick the cube up.")
                self.wait_for_pickup_command = False
            else:
                self.get_logger().warn("Invalid command, expecting: \npick color(optional)")
        elif self.wait_for_place_command:
            if content == "place":
                self.get_logger().info("Proceeding to place the cube.")
                self.wait_for_place_command = False
            else:
                self.get_logger().warn("Invalid command, expecting: place")
        else:
            pass #self.get_logger().warn("Warning: No command expected at this point")  TODO put back

    def run(self):
        # TODO replace all pass-es with spin once or async wait or whatever was recommended during the bootcamp
        while True:
            # State 0 - wait for pickup command:
            self.get_logger().info("Waiting for pick command")
            self.wait_for_pickup_command = True
            while(self.wait_for_pickup_command):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("State 0 done")
            # State 1 - goto initial arm position
            self.goto_position(self.init_position)
            self.get_logger().info("State 1 done")
            # State 2 - goto z,rho where feedback control can be turned on
            z = 0.175
            self.rho = 0.175
            try:
                joint2target, joint3target, joint4target = inverse_kinematics_to_joint_states(z=z,rho=self.rho)
            except:
                self.get_logger().warn("Inverse kinematics failed for z={}, rho={}, target might be unreachable".format(z,self.rho))
                # if it does fail here that rly sucks
            position = self.init_position[0],self.init_position[1],joint2target,joint3target,joint4target,self.init_position[5]
            self.goto_position(position)
            self.get_logger().info("State 2 done")
            # State 3 - feedback control ON, run until all errors are small, camera ON
            self.joint1target = self.init_position[1]
            self.joint2target = joint2target
            self.joint3target = joint3target
            self.joint4target = joint4target
            self.joint5target = self.init_position[5]
            self.z = z # make arm stay on this z while visual servoing
            self.cube_position_available = False
            self.sideways_integral_term = 0
            # Run visual servoing while the errors don't decrease, or a timeout doesnt trigger
            self.was_timed_out = False
            main_timeout = 10           # reset if visual servoing isnt complete in this time
            self.main_timeout_timer = self.create_timer(main_timeout, self.visual_servo_timeout)
            cant_see_cube_timeout = 2   # reset if cube cant be seen for this long while visual servoing
            self.cant_see_cube_timer = self.create_timer(cant_see_cube_timeout, self.visual_servo_timeout)
            self.visual_servoing_ON = True
            while self.visual_servoing_ON:
                rclpy.spin_once(self, timeout_sec=1)
            # Destroy timeout timers
            self.cant_see_cube_timer.destroy()      
            self.main_timeout_timer.destroy()
            if self.was_timed_out:     # If it was timed out, report back failure and go to the initial position
                self.report_pick_fail()
                self.get_logger().info("pick failed - timeout")
                self.goto_position(self.init_position)
                continue
            self.get_logger().info("State 3 done")
            # State 4 - feedback control OFF, goto lower z to pick up
            z = 0.14
            try:
                joint2target, joint3target, joint4target = inverse_kinematics_to_joint_states(z=z,rho=self.rho)
            except:
                self.get_logger().warn("Inverse kinematics failed for z={}, rho={}, target might be unreachable".format(z,self.rho))
            position = self.init_position[0],self.joint1target,joint2target,joint3target,joint4target,self.joint5target
            self.goto_position(position)
            self.get_logger().info("State 4 done")
            # State 5 - grip
            position = self.joint0grip_value,self.joint1target,joint2target,joint3target,joint4target,self.joint5target
            self.goto_position(position)
            self.get_logger().info("State 5 done")
            # State 6 - goto initial position but gripper closed, check whether pickup was successful, report back
            position = self.init_position.copy()
            position[0] = self.joint0grip_value
            self.goto_position(position)
            self.look_at_gripper_contents = True
            while self.look_at_gripper_contents:            # analyze a camera image in a callback
                rclpy.spin_once(self, timeout_sec=1)
            if self.cube_being_held:
                self.report_pick_success()
                self.get_logger().info("pick successful")
            else:
                self.report_pick_fail()
                self.get_logger().info("pick failed")
                self.goto_position(self.init_position)  # gripper release
                continue
            self.get_logger().info("State 6 done")
            # State 7 - wait for place command
            self.get_logger().info("Waiting for place command")
            self.wait_for_place_command = True
            while(self.wait_for_place_command):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("State 7 done")
            # State 8 - Gripper release, goto initial (state1?) position, report back
            self.goto_position(self.init_position)  # gripper release
            msg = String()
            msg.data = "place_success"
            self.report_publisher.publish(msg)
            self.get_logger().info("State 8 done")


    def goto_position(self,position):
        # Moves arm to hardcoded position in 1 sec
        assert(len(position) == 6)
        msg = ArmControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.time = [1000]*6
        msg.position = position
        self._pub_control.publish(msg)
        print(f"Going to position: {position}")
        time.sleep(1.5)

    def report_pick_success(self):
        msg = String()
        msg.data = "pick_success"
        self.report_publisher.publish(msg)

    def report_pick_fail(self):
        msg = String()
        msg.data = "pick_fail"
        self.report_publisher.publish(msg)

    def visual_servo_timeout(self):
        self.get_logger().warn("Visual servoing timed out (no cube seen or stuck for a long time)")
        self.was_timed_out = True
        self.visual_servoing_ON = False

    def timer_callback(self):
        # TODO put this entire thing into a separate function and maybe even file
        if self.visual_servoing_ON:
            if self.cube_position_available:
                try:
                    # Control gains:
                    k_sideways = 0.01#0.05                  commented values work with 0.5 sec timer
                    k_sideways_integral = 0.005#0.1
                    k_rotation = 1
                    k_extension = 0.001

                    # Previous targets:
                    prev_joint1target = self.joint1target
                    prev_joint2target = self.joint2target
                    prev_joint3target = self.joint3target
                    prev_joint4target = self.joint4target
                    prev_joint5target = self.joint5target                    

                    cx,cy = self.cube_position_in_frame
                    rotation = self.cube_orientation_in_frame
                    self.cube_position_available = False
                    # Calculate errors:
                    sideways_error = self.width_target-cx
                    rotation_error = rotation % 90
                    if rotation_error > 45:
                        rotation_error = rotation_error - 90
                    extension_error = self.height_target-cy
                    # Termination condition:
                    if (abs(sideways_error)<30) and (abs(rotation_error)<25) and (abs(extension_error)<50):
                        self.visual_servoing_ON = False
                        return
                        
                    # Sideways control (PI)
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
                    self.joint1target = 120 + k_rotation*rotation_error

                    # z-rho control (arm extend/contract + up-down, P)
                    self.rho = 0.175 + k_extension*extension_error
                    try:
                        self.joint2target, self.joint3target, self.joint4target = inverse_kinematics_to_joint_states(z=self.z,rho=self.rho)
                    except:
                        self.get_logger().warn("Inverse kinematics failed for z={}, rho={}, target might be unreachable".format(self.z,self.rho))

                    # Saturate each motor's speed
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
        
        
        
    def image_callback(self, msg: Image):
        # For each image received on /arm/camera/image_raw it updates the cube position and orientation variables
        # Known issues: it can detect multiple cubes/cube-like objects in the same frame, and both get written to the same attribute
        if self.visual_servoing_ON:
            try:
                self.cube_position_in_frame, self.cube_orientation_in_frame = find_cube_in_image_msg(msg, self._pub, self._pub2, self._pub3, self._pub4, self.width_target, self.height_target, publish_debug_images=True)
                self.cube_position_available = True
                self.cant_see_cube_timer.reset()    # keep reseting timeout timer while we see the cube
            except Exception as ex:     # if crash is due to no cube detected then pass, otherwise reraise
                if ex.args[0] != "Cube not found in frame":
                    raise ex
        elif self.look_at_gripper_contents:
            self.cube_being_held = is_the_target_cube_colored(msg, self.width_target, self.height_target, self._pub5)
            self.look_at_gripper_contents = False

                      


def main():
    rclpy.init()
    node = Arm_control()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
