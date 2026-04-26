#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool
from robp_interfaces.msg import DutyCycles
from grumpy_interfaces.msg import PathWithType


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PathControllerNode(Node):
    def __init__(self):
        super().__init__("path_controller_node")

        # Frames / timing
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 20.0)

        # Stop distances
        # self.declare_parameter("object_stop_distance", 0.185)
        # self.declare_parameter("box_stop_distance", 0.25)
        # self.declare_parameter("exploration_stop_distance", 0.10)

        self.declare_parameter("object_stop_distance", 0.15)
        self.declare_parameter("box_stop_distance", 0.15)
        self.declare_parameter("exploration_stop_distance", 0.15)

        # Path tracking
        self.declare_parameter("lookahead", 0.20)
        self.declare_parameter("advance_tolerance", 0.10)
        self.declare_parameter("angle_tolerance", 0.20)

        # Motion commands
        self.declare_parameter("forward_duty", 0.22)
        self.declare_parameter("turn_duty", 0.18)
        self.declare_parameter("min_forward_duty", 0.18)
        self.declare_parameter("min_turn_duty", 0.16)
        self.declare_parameter("max_duty", 0.35)

        # Slowdown near goal
        self.declare_parameter("slowdown_distance", 0.35)

        # Optional motor bias correction
        self.declare_parameter("left_scale", 1.0)
        self.declare_parameter("right_scale", 1.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(DutyCycles, "/phidgets/motor/duty_cycles", 10)
        self.reached_pub = self.create_publisher(Bool, "/nav/reached", 10)


        self.create_subscription(Bool, "/nav/path_blocked", self.on_path_blocked, 10)
        self.cancel_controller = False

        self.path_sub = self.create_subscription(
            PathWithType,
            "/nav/path_to_controller",
            self.on_path_with_type,
            10,
        )

        self.path = None
        self.next_idx = 0
        self.goal_type = PathWithType.OBJECT
        self.reached_latched = False
        self.stopped_latched = False

        rclpy.get_default_context().on_shutdown(self.stop)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info(
            "PathControllerNode up. "
            "Sub: /nav/path_to_controller  "
            "Pub: /phidgets/motor/duty_cycles, /nav/reached"
        )


    def on_path_blocked(self, msg):
        self.cancel_controller = msg.data

        if msg.data:
            self.path = None
            self.next_idx = 0
            self.reached_latched = False
            self.publish_reached(False)
            self.stop()
            self.get_logger().warn("Received /nav/path_blocked=True, stopping controller.")


    def goal_type_name(self) -> str:
        if self.goal_type == PathWithType.OBJECT:
            return "object"
        if self.goal_type == PathWithType.BOX:
            return "box"
        if self.goal_type == PathWithType.EXPLORATION_POINT:
            return "exploration_point"
        return f"unknown({self.goal_type})"

    def on_path_with_type(self, msg: PathWithType):

        self.get_logger().info("on_path_with_type")

        if not msg.path.poses:
            self.path = None
            self.next_idx = 0
            self.reached_latched = False
            self.get_logger().info("Received empty path. Stopping.")
            return

        if msg.path.header.frame_id and msg.path.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f"Path frame '{msg.path.header.frame_id}' != '{self.world_frame}'. "
                f"Publish path in {self.world_frame}."
            )
            return

        self.path = msg.path
        self.goal_type = msg.type
        self.next_idx = 0
        self.reached_latched = False
        self.stopped_latched = False

        self.get_logger().info(
            f"Received new path with {len(msg.path.poses)} poses, "
            f"type={self.goal_type_name()}."
        )

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().info(f"Transform lookup error: {e}", throttle_duration_sec=1.0)
            return None

        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        return t.x, t.y, yaw

    def publish_duty(self, left: float, right: float):
        # self.get_logger().info(f"publishin duty = ({left}, {right})")
        left_scale = float(self.get_parameter("left_scale").value)
        right_scale = float(self.get_parameter("right_scale").value)
        max_duty = float(self.get_parameter("max_duty").value)

        left = clamp(left * left_scale, -max_duty, max_duty)
        right = clamp(right * right_scale, -max_duty, max_duty)

        msg = DutyCycles()
        msg.duty_cycle_left = float(left)
        msg.duty_cycle_right = float(right)
        self.cmd_pub.publish(msg)

    def publish_reached(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.reached_pub.publish(msg)

    def allow_motion(self):
        self.stopped_latched = False

    def stop(self):
        if self.stopped_latched:
            return

        self.publish_duty(0.0, 0.0)
        self.stopped_latched = True

    def stop_distance(self) -> float:
        if self.goal_type == PathWithType.BOX:
            return float(self.get_parameter("box_stop_distance").value)
        if self.goal_type == PathWithType.EXPLORATION_POINT:
            return float(self.get_parameter("exploration_stop_distance").value)
        return float(self.get_parameter("object_stop_distance").value)

    def step(self):
        if self.path is None:
            self.stop()
            # self.publish_reached(False)
            return
        #
        if self.cancel_controller:
            self.stop()
            # self.publish_reached(False)
            return

        pose = self.get_pose()
        if pose is None:
            self.stop()
            return

        x, y, yaw = pose

        stop_dist = self.stop_distance()
        lookahead = float(self.get_parameter("lookahead").value)
        advance_tol = float(self.get_parameter("advance_tolerance").value)
        angle_tol = float(self.get_parameter("angle_tolerance").value)

        base_forward = float(self.get_parameter("forward_duty").value)
        base_turn = float(self.get_parameter("turn_duty").value)
        min_forward = float(self.get_parameter("min_forward_duty").value)
        min_turn = float(self.get_parameter("min_turn_duty").value)
        slowdown_distance = float(self.get_parameter("slowdown_distance").value)

        # Final goal distance
        final = self.path.poses[-1].pose.position
        d_final = math.hypot(final.x - x, final.y - y)

        # Stop when inside standoff distance
        if d_final <= stop_dist:
            self.stop()

            if not self.reached_latched:
                self.publish_reached(True)
                self.reached_latched = True
                self.get_logger().info(
                    f"Reached ({self.goal_type_name()}) within {stop_dist:.2f} m (d={d_final:.2f})."
                )
            self.path = None
            self.next_idx = 0
            return
        # else:
        #     self.publish_reached(False)
        #     self.reached_latched = False

        # Advance along the path
        # while self.next_idx + 1 < len(self.path.poses):
        #     p = self.path.poses[self.next_idx].pose.position
        #     d = math.hypot(p.x - x, p.y - y)
        #     if d < advance_tol:
        #         self.next_idx += 1
        #     else:
        #         break

        # Advance along the path by snapping to the closest point ahead
        closest_idx = self.next_idx
        closest_dist = float("inf")

        for i in range(self.next_idx, len(self.path.poses)):
            p = self.path.poses[i].pose.position
            d = math.hypot(p.x - x, p.y - y)

            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        if closest_idx > self.next_idx:
            self.next_idx = closest_idx

        # Also skip points we are already close to
        while self.next_idx + 1 < len(self.path.poses):
            p = self.path.poses[self.next_idx].pose.position
            d = math.hypot(p.x - x, p.y - y)

            if d < advance_tol:
                self.next_idx += 1
            else:
                break

        # self.get_logger().info(
        #     f"idx={self.next_idx}, target_idx={target_idx}, d_final={d_final:.2f}, err={err:.2f}",
        #     throttle_duration_sec=0.5,
        # )

        # Choose lookahead target
        target_idx = len(self.path.poses) - 1
        for i in range(self.next_idx, len(self.path.poses)):
            p = self.path.poses[i].pose.position
            d = math.hypot(p.x - x, p.y - y)
            if d >= lookahead:
                target_idx = i
                break

        target = self.path.poses[target_idx].pose.position
        dx = target.x - x
        dy = target.y - y

        desired_yaw = math.atan2(dy, dx)
        err = wrap_pi(desired_yaw - yaw)

        # Slow down near final goal, but never below physical minimum
        if d_final <= stop_dist + slowdown_distance:
            ratio = clamp((d_final - stop_dist) / slowdown_distance, 0.0, 1.0)
            forward_cmd = max(min_forward, base_forward * ratio)
            turn_cmd = max(min_turn, base_turn * ratio)
        else:
            forward_cmd = base_forward
            turn_cmd = base_turn

        # If heading error is too large -> turn in place
        # Otherwise -> drive forward
        if abs(err) > angle_tol:
            if err > 0.0:
                left = -turn_cmd
                right = +turn_cmd
            else:
                left = +turn_cmd
                right = -turn_cmd
        else:
            left = forward_cmd
            right = forward_cmd

        self.allow_motion()
        self.publish_duty(left, right)


def main():
    rclpy.init()
    node = PathControllerNode()

    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
