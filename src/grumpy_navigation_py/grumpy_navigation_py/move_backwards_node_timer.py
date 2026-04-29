#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robp_interfaces.msg import DutyCycles


class MoveBackwardsTimed(Node):
    def __init__(self):
        super().__init__("move_backwards_timed")

        self.declare_parameter("backwards_duty", 0.10)
        self.declare_parameter("duration", 2.0)  # seconds
        self.declare_parameter("max_duty", 0.20)

        self.moving = False

        self.create_subscription(
            Bool,
            "/nav/move_backwards_start",
            self.start_callback,
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

        self.timer = None

    def start_callback(self, msg):
        if not msg.data or self.moving:
            return

        self.get_logger().info("Start moving backwards")

        self.moving = True

        duty = float(self.get_parameter("backwards_duty").value)
        self.publish_duty(-duty, -duty)

        duration = float(self.get_parameter("duration").value)
        self.timer = self.create_timer(duration, self.stop_callback)

    def stop_callback(self):
        self.stop()
        self.moving = False

        done = Bool()
        done.data = True
        self.finished_pub.publish(done)

        self.get_logger().info("Finished moving backwards")

        self.timer.cancel()
        self.timer = None

    def stop(self):
        self.publish_duty(0.0, 0.0)

    def publish_duty(self, left, right):
        max_duty = float(self.get_parameter("max_duty").value)

        msg = DutyCycles()
        msg.duty_cycle_left = max(min(left, max_duty), -max_duty)
        msg.duty_cycle_right = max(min(right, max_duty), -max_duty)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MoveBackwardsTimed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
