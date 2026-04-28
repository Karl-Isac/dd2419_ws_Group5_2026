#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robp_interfaces.msg import DutyCycles

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class MoveBackWardsNode(Node):
    def __init__(self):
        super().__init__("move_backwards_node")

        self.declare_parameter("backwards_duty", 0.10)
        self.declare_parameter("backwards_distance", 0.15)
        self.declare_parameter("max_duty", 0.20)
        self.declare_parameter("left_scale", 1.0)
        self.declare_parameter("right_scale", 1.0)

        self.base_frame = "base_link"
        self.world_frame = "map"

        self.start_x = None
        self.start_y = None
        self.moving = False

        # self.create_subscription(Bool, "/nav/move_backwards_start", self.on_move_backwards_start, 10)
        self.create_subscription(Header, "/nav/move_backwards_start", self.on_move_backwards_start, 10)
        self.move_backwards_finished_pub = self.create_publisher(Bool, "/nav/move_backwards_finished", 10)

        self.cmd_pub = self.create_publisher(DutyCycles, "/phidgets/motor/duty_cycles", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.on_timer)

    def on_move_backwards_start(self, msg):
        self.get_logger().info("on_move_backwards_start")

        # if not msg.data:
        #     return

        pose = self.get_robot_xy(msg.stamp)
        if pose is None:
            self.get_logger().warn("Could not get robot pose, cannot move backwards")
            return

        self.start_x, self.start_y = pose
        self.moving = True
        self.get_logger().info("Moving backwards")

    def on_timer(self):
        if not self.moving:
            return

        pose = self.get_robot_xy()
        if pose is None:
            self.stop()
            return

        x, y = pose
        dist = math.hypot(x - self.start_x, y - self.start_y)

        target_dist = float(self.get_parameter("backwards_distance").value)

        if dist >= target_dist:
            self.stop()
            self.moving = False

            msg = Bool()
            msg.data = True
            self.move_backwards_finished_pub.publish(msg)

            self.get_logger().info("Finished moving backwards")
            return

        duty = float(self.get_parameter("backwards_duty").value)

        # If positive duty moves forward on your robot, change this to (-duty, -duty)
        self.publish_duty(-duty, -duty)

    def get_robot_xy(self, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y

        # except (LookupException, ConnectivityException, ExtrapolationException):

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            # self.rerun_on_goal_later = True
            self.rerun_on_backup_timer = self.create_timer(0.1, self.rerun_on_move_backwards_callback)
            return None

    def rerun_on_move_backwards_callback(self):
        self.get_logger().info("rerun_on_move_backwards_callback")
        self.rerun_on_backup_timer.destroy() 
        self.on_goal(self.latest_on_goal_message)

    def stop(self):
        self.publish_duty(0.0, 0.0)

    def publish_duty(self, left: float, right: float):
        # self.get_logger().info(f"publish duty: ({left}, {right})")
        left_scale = float(self.get_parameter("left_scale").value)
        right_scale = float(self.get_parameter("right_scale").value)
        max_duty = float(self.get_parameter("max_duty").value)

        left = clamp(left * left_scale, -max_duty, max_duty)
        right = clamp(right * right_scale, -max_duty, max_duty)

        msg = DutyCycles()
        msg.duty_cycle_left = float(left)
        msg.duty_cycle_right = float(right)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MoveBackWardsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
