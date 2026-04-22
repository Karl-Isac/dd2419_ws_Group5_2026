#!/usr/bin/env python

import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped, PoseStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        self._path = Path()

        # Encoder subscription
        self.create_subscription(
            Encoders,
            '/phidgets/motor/encoders',
            self.encoder_callback,
            10
        )

        # Pose state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.get_logger().info("Odometry (ENCODER ONLY) started")

    def encoder_callback(self, msg: Encoders):
        # --- Parameters (same as your working version) ---
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04921
        base = 0.315

        # Encoder deltas
        dL = msg.delta_encoder_left
        dR = msg.delta_encoder_right

        # Convert ticks → radians
        K = 2 * math.pi / ticks_per_rev

        # Distance and rotation
        D = K * wheel_radius / 2.0 * (dR + dL)
        dTheta = K * wheel_radius / base * (dR - dL)

        # --- Better integration (midpoint) ---
        yaw_mid = self._yaw + dTheta / 2.0

        self._x += D * math.cos(yaw_mid)
        self._y += D * math.sin(yaw_mid)
        self._yaw += dTheta

        # Normalize yaw
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_path(stamp, self._x, self._y, self._yaw)

    def broadcast_transform(self, stamp, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
