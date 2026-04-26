#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ApproachGoalNode(Node):
    def __init__(self):
        super().__init__("approach_goal_node")

        # Frames / timing
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 20.0)

        # Turning behavior
        self.declare_parameter("angle_tolerance", 0.10)   # rad
        self.declare_parameter("turn_duty", 0.10)

        # Forward behavior
        # self.declare_parameter("forward_distance", 0.07)  # meters
        self.declare_parameter("stop_distance", 0.19)
        self.declare_parameter("forward_duty", 0.10)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            "/nav/approach_start",
            self.on_approach_start,
            10,
        )

        self.approach_finished_pub = self.create_publisher(
            PoseStamped,
            "/nav/approach_finished",
            10,
        )
        self.duty_pub = self.create_publisher(
            DutyCycles,
            "/phidgets/motor/duty_cycles",
            10,
        )

        # State
        self.state = "IDLE"   # IDLE / TURNING / FORWARD
        self.target_pose = None
        self.forward_start_xy = None

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        rclpy.get_default_context().on_shutdown(self.stop)

        self.get_logger().info("ApproachGoalNode up")

    def on_approach_start(self, msg: PoseStamped):
        self.target_pose = msg
        self.forward_start_xy = None
        self.state = "TURNING"
        self.get_logger().info(
            f"Received approach target ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

    def lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        return float(t.x), float(t.y), float(yaw)

    def publish_duty(self, left: float, right: float):
        # self.get_logger().info(f"publsihing duty: ({left}, {right})")
        msg = DutyCycles()
        msg.duty_cycle_left = float(left)
        msg.duty_cycle_right = float(right)
        self.duty_pub.publish(msg)

    def stop(self):
        self.publish_duty(0.0, 0.0)

    def publish_finished(self):
        if self.target_pose is None:
            return

        finished = PoseStamped()
        finished.header.frame_id = self.world_frame
        finished.header.stamp = self.get_clock().now().to_msg()
        finished.pose = self.target_pose.pose
        self.approach_finished_pub.publish(finished)

    def step(self):
        if self.state == "IDLE":
            return

        pose = self.lookup_pose()
        if pose is None or self.target_pose is None:
            self.stop()
            return

        x, y, yaw = pose
        tx = float(self.target_pose.pose.position.x)
        ty = float(self.target_pose.pose.position.y)

        if self.state == "TURNING":
            desired_yaw = math.atan2(ty - y, tx - x)
            yaw_error = wrap_pi(desired_yaw - yaw)

            angle_tolerance = float(self.get_parameter("angle_tolerance").value)
            turn_duty = float(self.get_parameter("turn_duty").value)

            if abs(yaw_error) <= angle_tolerance:
                self.stop()
                self.forward_start_xy = (x, y)
                self.state = "FORWARD"
                self.get_logger().info("Approach turn finished, starting forward motion")
                return

            if yaw_error > 0.0:
                left = -turn_duty
                right = +turn_duty
            else:
                left = +turn_duty
                right = -turn_duty

            self.publish_duty(left, right)
            return

        if self.state == "FORWARD":
            if self.forward_start_xy is None:
                self.forward_start_xy = (x, y)

            # sx, sy = self.forward_start_xy
            # traveled = math.hypot(x - sx, y - sy)
            #
            # forward_distance = float(self.get_parameter("forward_distance").value)
            # forward_duty = float(self.get_parameter("forward_duty").value)
            #
            # if traveled >= forward_distance:

            dist_to_target = math.hypot(tx - x, ty - y)

            stop_distance = float(self.get_parameter("stop_distance").value)
            forward_duty = float(self.get_parameter("forward_duty").value)

            if dist_to_target <= stop_distance:
                self.stop()
                self.publish_finished()
                self.state = "IDLE"
                self.target_pose = None
                self.forward_start_xy = None
                self.get_logger().info("Approach finished")
                return

            self.publish_duty(forward_duty, forward_duty)


def main():
    rclpy.init()
    node = ApproachGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
