#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robp_interfaces.msg import DutyCycles, Encoders


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class MoveBackwardsEncoderNode(Node):
    def __init__(self):
        super().__init__("move_backwards_encoder_node")

        self.declare_parameter("backwards_duty", 0.10)
        self.declare_parameter("backwards_distance", 0.15)  # meters
        self.declare_parameter("max_duty", 0.20)

        # CHANGE THESE TO YOUR ROBOT VALUES
        self.declare_parameter("ticks_per_rev", 3072.0)
        self.declare_parameter("wheel_radius", 0.049)  # meters

        self.moving = False
        self.distance_moved = 0.0

        self.create_subscription(
            Bool,
            "/nav/move_backwards_start",
            self.start_callback,
            10,
        )

        self.create_subscription(
            Encoders,
            "/phidgets/motor/encoders",
            self.encoder_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            DutyCycles,
            "/phidgets/motor/duty_cycles",
            10,
        )

        self.finished_pub = self.create_publisher(
            Bool,
            "/nav/move_backwards_finished",
            10,
        )

    def start_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info("Moving backwards")
        self.distance_moved = 0.0
        self.moving = True

        duty = float(self.get_parameter("backwards_duty").value)
        self.publish_duty(-duty, -duty)

    def encoder_callback(self, msg):
        if not self.moving:
            return

        # Adjust field names if your Encoders msg uses different names
        left_ticks = msg.delta_encoder_left
        right_ticks = msg.delta_encoder_right

        ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        wheel_radius = float(self.get_parameter("wheel_radius").value)

        meters_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev

        left_dist = left_ticks * meters_per_tick
        right_dist = right_ticks * meters_per_tick

        # Use abs because we only care how far we backed up
        ds = abs((left_dist + right_dist) / 2.0)
        self.distance_moved += ds

        target = float(self.get_parameter("backwards_distance").value)

        if self.distance_moved >= target:
            self.stop()
            self.moving = False

            done = Bool()
            done.data = True
            self.finished_pub.publish(done)

            self.get_logger().info("Finished moving backwards")

    def stop(self):
        self.publish_duty(0.0, 0.0)

    def publish_duty(self, left, right):
        max_duty = float(self.get_parameter("max_duty").value)

        msg = DutyCycles()
        msg.duty_cycle_left = clamp(left, -max_duty, max_duty)
        msg.duty_cycle_right = clamp(right, -max_duty, max_duty)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MoveBackwardsEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
