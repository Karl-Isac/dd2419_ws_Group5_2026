
#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node

import tf2_ros

from robp_interfaces.msg import DutyCycles


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q) -> float:
    # geometry_msgs/Quaternion: x,y,z,w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RandomNavNode(Node):
    def __init__(self):
        super().__init__("random_nav_node")

        # Rectangle bounds (in the same frame you use for pose, e.g. "odom" or "map")
        self.declare_parameter("xmin", -0.5)
        self.declare_parameter("xmax", 0.5)
        self.declare_parameter("ymin", -0.5)
        self.declare_parameter("ymax", 0.5)

        # Frames: choose what pose frame you want to navigate in
        # For MS1, "odom" is usually fine (no localization needed).
        self.declare_parameter("world_frame", "odom")     # "odom" or "map"
        self.declare_parameter("base_frame", "base_link") # usually "base_link"

        # Goal + control
        self.declare_parameter("reach_tolerance", 0.25)  # m
        self.declare_parameter("angle_tolerance", 0.25)  # rad
        self.declare_parameter("rate_hz", 20.0)

        # Duty-cycle commands (keep it simple)
        self.declare_parameter("duty_forward", 0.2)  # 0..1 typical (confirm for your robot)
        self.declare_parameter("duty_turn", 0.15)     # turning duty

        # self.cmd_pub = self.create_publisher(DutyCycles, "/wheel_duty_cycles", 10)
        self.cmd_pub = self.create_publisher(DutyCycles, "/phidgets/motor/duty_cycles", 10)

        # TF for pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.goal = None

        rclpy.get_default_context().on_shutdown(self.stop)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.sample_goal()

    def sample_goal(self):
        xmin = float(self.get_parameter("xmin").value)
        xmax = float(self.get_parameter("xmax").value)
        ymin = float(self.get_parameter("ymin").value)
        ymax = float(self.get_parameter("ymax").value)
        self.goal = (random.uniform(xmin, xmax), random.uniform(ymin, ymax))
        self.get_logger().info(f"New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def get_pose(self):
        """Return (x, y, yaw) in world_frame, or None if TF not ready."""
        world = self.get_parameter("world_frame").value
        base = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(world, base, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f"transform lookup error: {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = quat_to_yaw(q)
        return (t.x, t.y, yaw)

    def publish_duty(self, left: float, right: float):
        msg = DutyCycles()
        msg.duty_cycle_left = float(left)
        msg.duty_cycle_right = float(right)
        self.cmd_pub.publish(msg)

    def stop(self):
        #print("stop function")
        self.get_logger().info("stop function")
        self.publish_duty(0.0, 0.0)

    def step(self):
        if self.goal is None:
            return

        pose = self.get_pose()
        if pose is None:
            # TF not ready yet, skip this tick quietly
            return

        x, y, yaw = pose

        #self.get_logger().info(
        #    f"{self.get_parameter('world_frame').value} -> {self.get_parameter('base_frame').value}" + "\n" +
        #    f"x:{x}, y:{y}, yaw{yaw}",
        #    throttle_duration_sec=1.0,
        #)

        gx, gy = self.goal

        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)

        if dist < float(self.get_parameter("reach_tolerance").value):
            self.stop()
            self.sample_goal()
            return

        target = math.atan2(dy, dx)
        err = wrap_pi(target - yaw)

        angle_tol = float(self.get_parameter("angle_tolerance").value)
        duty_fwd = float(self.get_parameter("duty_forward").value)
        duty_turn = float(self.get_parameter("duty_turn").value)

        # Spin-then-straight (minimal MS1 behavior)
        if abs(err) > angle_tol:
            if err > 0.0:
                # turn left in place
                self.publish_duty(-duty_turn, +duty_turn)
            else:
                # turn right in place
                self.publish_duty(+duty_turn, -duty_turn)
        else:
            # drive forward
            self.publish_duty(+duty_fwd, +duty_fwd)


def main():
    rclpy.init()
    node = RandomNavNode()
    #print("hello")
    try:
        rclpy.spin(node)
    #except KeyboardInterrupt as ki:
    #    node.stop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
