#!/usr/bin/env python3

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

        self._tf_broadcaster = TransformBroadcaster(self)

        self._path_pub = self.create_publisher(Path, 'path', 10)
        self._path = Path()

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
        self._ticks_per_rev = 50 * 64
        self._wheel_radius = 0.04921
        self._base = 0.315  # distance between wheels

    def encoder_callback(self, msg: Encoders):
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        # Wheel angle increments
        phi_L = (delta_ticks_left / self._ticks_per_rev) * 2.0 * math.pi
        phi_R = (delta_ticks_right / self._ticks_per_rev) * 2.0 * math.pi

        # Distance traveled by each wheel
        dL = self._wheel_radius * phi_L
        dR = self._wheel_radius * phi_R

        # Robot motion
        D = 0.5 * (dR + dL)
        d_yaw = (dR - dL) / self._base

        # Better integration: use midpoint heading
        yaw_mid = self._yaw + 0.5 * d_yaw
        self._x += D * math.cos(yaw_mid)
        self._y += D * math.sin(yaw_mid)
        self._yaw += d_yaw

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
