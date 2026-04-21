#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)

        # Store the path here
        self._path = Path()
        self._path.header.frame_id = 'odom'

        # Subscribe to encoder topic
        self.create_subscription(
            Encoders,
            '/phidgets/motor/encoders',
            self.encoder_callback,
            10
        )

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Robot parameters
        self.ticks_per_rev = 48 * 64
        self.wheel_radius = 0.04921
        self.base = 0.315

        # Publish initial pose immediately
        stamp = self.get_clock().now().to_msg()
        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_path(stamp, self._x, self._y, self._yaw)

        self.get_logger().info("odometry node is up")

    def encoder_callback(self, msg: Encoders):
        """Update odometry from encoder delta ticks."""

        # Ticks since last message
        dL = msg.delta_encoder_left
        dR = msg.delta_encoder_right

        # Convert ticks to traveled distance
        K = 2.0 * math.pi / self.ticks_per_rev
        D = K * self.wheel_radius * 0.5 * (dR + dL)
        dTheta = K * self.wheel_radius / self.base * (dR - dL)

        # Midpoint integration for better accuracy
        self._x += D * math.cos(self._yaw + dTheta / 2.0)
        self._y += D * math.sin(self._yaw + dTheta / 2.0)
        self._yaw += dTheta

        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_path(stamp, self._x, self._y, self._yaw)

    def broadcast_transform(self, stamp, x, y, yaw):
        """Broadcast odom -> base_link transform."""

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
        """Append current pose to path and publish it."""

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
