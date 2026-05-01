#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, PoseStamped
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


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

        self.create_subscription(
            Imu,
            '/phidgets/imu/data_raw',
            self.imu_callback,
            10
        )

        # ---------------- state ----------------
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0   # ⭐唯一状态

        # ---------------- IMU ----------------
        self._omega_imu = 0.0
        self._yaw_meas = 0.0
        self._IMU_offset = None

        # ---------------- time ----------------
        self._encoder_time = None

        # complementary filter gain
        self.alpha = 0.98

    # =========================================================
    # IMU callback (measurement only)
    # =========================================================
    def imu_callback(self, msg: Imu):

        # gyro z
        self._omega_imu = msg.angular_velocity.z

        # orientation yaw
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self._IMU_offset is None:
            self._IMU_offset = yaw

        self._yaw_meas = wrap_angle(yaw - self._IMU_offset)

    # =========================================================
    # Encoder + Fusion (state update here)
    # =========================================================
    def encoder_callback(self, msg: Encoders):

        t = Time.from_msg(msg.header.stamp)

        if self._encoder_time is None:
            self._encoder_time = t
            return

        dt = (t - self._encoder_time).nanoseconds * 1e-9
        self._encoder_time = t

        if dt <= 0.0:
            return

        # ---------------- wheel model ----------------
        ticks_per_rev = 50 * 64
        wheel_radius = 0.04921

        dL = msg.delta_encoder_left
        dR = msg.delta_encoder_right

        phi_L = (dL / ticks_per_rev) * 2 * math.pi
        phi_R = (dR / ticks_per_rev) * 2 * math.pi

        D = wheel_radius / 2.0 * (phi_R + phi_L)

        # =========================================================
        # complementary filter (fusion)
        # =========================================================

        # prediction from gyro
        yaw_pred = self._yaw + self._omega_imu * dt

        # fusion with IMU absolute yaw
        self._yaw = self.alpha * yaw_pred + (1.0 - self.alpha) * self._yaw_meas
        self._yaw = wrap_angle(self._yaw)

        # =========================================================
        # position update
        # =========================================================
        self._x += D * math.cos(self._yaw)
        self._y += D * math.sin(self._yaw)

        # publish
        stamp = msg.header.stamp
        self.broadcast_transform(stamp, self._x, self._y, -self._yaw)
        self.publish_path(stamp, self._x, self._y, -self._yaw)

    # =========================================================
    # TF
    # =========================================================
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

    # =========================================================
    # Path
    # =========================================================
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