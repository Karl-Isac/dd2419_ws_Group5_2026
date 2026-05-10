#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


class RotateToPoseNode(Node):
    def __init__(self):
        super().__init__("rotate_to_pose_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("yaw_tolerance", 0.05)      # rad
        self.declare_parameter("kp", 1.5)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.kp = float(self.get_parameter("kp").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.target_yaw = None
        self.active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.finished_pub = self.create_publisher(Bool, "/nav/rotate_finished", 10)

        self.create_subscription(
            PoseStamped,
            "/nav/rotate_to_pose",
            self.on_rotate_goal,
            10
        )

        dt = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info("RotateToPoseNode up.")

    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def on_rotate_goal(self, msg: PoseStamped):
        self.target_yaw = self.yaw_from_quat(msg.pose.orientation)
        self.active = True

        self.get_logger().info(
            f"Received rotate goal yaw={self.target_yaw:.3f} rad"
        )

    def get_current_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        return self.yaw_from_quat(tf.transform.rotation)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.active or self.target_yaw is None:
            return

        current_yaw = self.get_current_yaw()
        if current_yaw is None:
            return

        error = self.normalize_angle(self.target_yaw - current_yaw)

        if abs(error) < self.yaw_tolerance:
            self.stop_robot()
            self.active = False
            self.target_yaw = None
            self.finished_pub.publish(Bool(data=True))
            self.get_logger().info("Rotation finished")
            return

        cmd = Twist()
        wz = self.kp * error
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))
        cmd.angular.z = wz

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RotateToPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
