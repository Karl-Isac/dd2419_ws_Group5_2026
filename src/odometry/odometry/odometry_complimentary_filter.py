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

        # ================= STATE =================
        self._x = 0.0
        self._y = 0.0

        # fused yaw（最终用这个）
        self._yaw = 0.0

        # ================= IMU =================
        self._imu_last_time = None
        self._gyro_z = 0.0
        self._gyro_bias = 0.0
        self._bias_buffer = []

        self._yaw_imu = 0.0   # IMU积分yaw

        # ================= ENCODER =================
        self._encoder_time = None
        self._yaw_enc = 0.0

        # ================= TUNING =================
        self.alpha = 0.01   # encoder → IMU correction gain (关键参数)

    # =========================================================
    # IMU callback (gyro integration + bias estimation)
    # =========================================================
    def imu_callback(self, msg: Imu):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        gyro_z = msg.angular_velocity.z

        # -------- bias estimation (startup only) --------
        if len(self._bias_buffer) < 300:
            self._bias_buffer.append(gyro_z)
            if len(self._bias_buffer) == 300:
                self._gyro_bias = sum(self._bias_buffer) / len(self._bias_buffer)
                self.get_logger().warn(f"[IMU] bias = {self._gyro_bias:.6f}")
            return

        gyro = gyro_z - self._gyro_bias

        # -------- time integration --------
        if self._imu_last_time is None:
            self._imu_last_time = t
            return

        dt = t - self._imu_last_time
        self._imu_last_time = t

        if dt <= 0.0 or dt > 0.1:
            return

        # -------- IMU integration --------
        self._yaw_imu += gyro * dt
        self._yaw_imu = wrap_angle(self._yaw_imu)

    # =========================================================
    # Encoder callback (slow but drift-free reference)
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

        # ================= wheel params =================
        ticks_per_rev = 50 * 64
        wheel_radius = 0.04921
        wheel_base = 0.315  

        dL = msg.delta_encoder_left
        dR = msg.delta_encoder_right

        phi_L = (dL / ticks_per_rev) * 2 * math.pi
        phi_R = (dR / ticks_per_rev) * 2 * math.pi

        # -------- forward + yaw from encoder --------
        D = wheel_radius / 2.0 * (phi_R + phi_L)
        dtheta_enc = (wheel_radius / wheel_base) * (phi_R - phi_L)

        # update encoder yaw
        self._yaw_enc += dtheta_enc
        self._yaw_enc = wrap_angle(self._yaw_enc)

        # =========================================================
        # 🔥 KEY: encoder → IMU correction (drift suppression)
        # =========================================================
        error = wrap_angle(self._yaw_enc - self._yaw_imu)

        self._yaw_imu += self.alpha * error
        self._yaw_imu = wrap_angle(self._yaw_imu)

        # =========================================================
        # final fused yaw
        # =========================================================
        self._yaw = self._yaw_imu

        # position update
        self._x += D * math.cos(self._yaw)
        self._y += D * math.sin(self._yaw)

        # publish
        self.broadcast_transform(msg.header.stamp, self._x, self._y, -self._yaw)
        self.publish_path(msg.header.stamp, self._x, self._y, -self._yaw)

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