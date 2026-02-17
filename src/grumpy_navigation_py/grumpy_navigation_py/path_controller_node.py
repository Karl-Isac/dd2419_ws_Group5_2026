
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros

from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from robp_interfaces.msg import DutyCycles


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

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 20.0)

        # Different stopping distances per phase
        self.declare_parameter("object_stop_distance", 0.35)  # stop standoff for pickup
        self.declare_parameter("box_stop_distance", 0.25)     # stop standoff for dropoff

        # Tracking / steering
        self.declare_parameter("lookahead", 0.40)
        self.declare_parameter("angle_tolerance", 0.25)

        # Motor output
        self.declare_parameter("duty_forward", 0.20)
        self.declare_parameter("duty_turn", 0.15)
        self.declare_parameter("max_duty", 0.35)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(DutyCycles, "/phidgets/motor/duty_cycles", 10)
        self.reached_pub = self.create_publisher(Bool, "/nav/reached", 10)

        self.path_sub = self.create_subscription(Path, "/nav/path", self.on_path, 10)
        self.phase_sub = self.create_subscription(String, "/nav/phase", self.on_phase, 10)

        self.path = None
        self.next_idx = 0

        self.phase = "object"  # default
        self.reached_latched = False

        rclpy.get_default_context().on_shutdown(self.stop)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info("PathControllerNode up. Sub: /nav/path, /nav/phase  Pub: duty_cycles, /nav/reached")

    def on_phase(self, msg: String):
        p = (msg.data or "").strip().lower()
        if p not in ("object", "box"):
            self.get_logger().warn(f"/nav/phase must be 'object' or 'box', got '{msg.data}'")
            return
        if p != self.phase:
            self.get_logger().info(f"Phase changed: {self.phase} -> {p}")
        self.phase = p
        # When phase changes, allow reaching again
        self.reached_latched = False

    def on_path(self, msg: Path):
        if not msg.poses:
            self.path = None
            self.next_idx = 0
            self.reached_latched = False
            return
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f"Path frame '{msg.header.frame_id}' != '{self.world_frame}'. Publish path in {self.world_frame}."
            )
            return
        self.path = msg
        self.next_idx = 0
        self.reached_latched = False  # new plan => can reach again

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f"transform lookup error: {e}", throttle_duration_sec=1.0)
            return None
        t = tf.transform.translation
        yaw = quat_to_yaw(tf.transform.rotation)
        return (t.x, t.y, yaw)

    def publish_duty(self, left: float, right: float):
        msg = DutyCycles()
        msg.duty_cycle_left = float(left)
        msg.duty_cycle_right = float(right)
        self.cmd_pub.publish(msg)

    def publish_reached(self, value: bool):
        b = Bool()
        b.data = bool(value)
        self.reached_pub.publish(b)

    def stop(self):
        self.publish_duty(0.0, 0.0)

    def stop_distance(self) -> float:
        if self.phase == "box":
            return float(self.get_parameter("box_stop_distance").value)
        return float(self.get_parameter("object_stop_distance").value)

    def step(self):
        if self.path is None:
            self.stop()
            self.publish_reached(False)
            return

        pose = self.get_pose()
        if pose is None:
            return

        x, y, yaw = pose

        stop_dist = self.stop_distance()
        lookahead = float(self.get_parameter("lookahead").value)

        # Distance to FINAL goal (we stop at standoff, not exactly on it)
        final = self.path.poses[-1].pose.position
        d_final = math.hypot(final.x - x, final.y - y)

        if d_final <= stop_dist:
            self.stop()
            if not self.reached_latched:
                self.publish_reached(True)
                self.reached_latched = True
                self.get_logger().info(
                    f"Reached ({self.phase}) within {stop_dist:.2f} m (d={d_final:.2f}). Stopping."
                )
            return
        else:
            # Not reached: publish False (or keep last; I prefer explicit False)
            self.publish_reached(False)
            self.reached_latched = False

        # Advance next_idx a bit (use a smaller tolerance than stop_dist so we still progress)
        advance_tol = min(0.20, 0.5 * stop_dist)
        while self.next_idx + 1 < len(self.path.poses):
            p = self.path.poses[self.next_idx].pose.position
            if math.hypot(p.x - x, p.y - y) < advance_tol:
                self.next_idx += 1
            else:
                break

        # Pick lookahead target
        target_idx = len(self.path.poses) - 1
        for i in range(self.next_idx, len(self.path.poses)):
            p = self.path.poses[i].pose.position
            if math.hypot(p.x - x, p.y - y) >= lookahead:
                target_idx = i
                break

        target = self.path.poses[target_idx].pose.position
        dx, dy = target.x - x, target.y - y
        dist = math.hypot(dx, dy)

        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - yaw)

        angle_tol = float(self.get_parameter("angle_tolerance").value)
        duty_fwd = float(self.get_parameter("duty_forward").value)
        duty_turn = float(self.get_parameter("duty_turn").value)
        max_duty = float(self.get_parameter("max_duty").value)

        # Slow down as we approach stop distance to avoid overshoot
        # When d_final == stop_dist => scale ~0
        slow_band = max(0.10, 0.50)  # meters of slowdown band
        scale = clamp((d_final - stop_dist) / slow_band, 0.0, 1.0)

        if abs(err) > angle_tol:
            if err > 0.0:
                left, right = -duty_turn, +duty_turn
            else:
                left, right = +duty_turn, -duty_turn
        else:
            left = duty_fwd * scale
            right = duty_fwd * scale

        self.publish_duty(
            clamp(left, -max_duty, max_duty),
            clamp(right, -max_duty, max_duty),
        )


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
