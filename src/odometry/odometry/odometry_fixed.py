#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, PoseStamped
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
import time


class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()

        # Subscribe to encoder topic and call callback function on each recieved message
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

        self._yaw_imu = 0.0
        self._IMU_offset = None
        self._start_offset = 0  # If start yaw is not 0, change it here
        # self._start_offset = -3.2428191 # For Lidar_bag

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Ignore first encoder message so odom starts cleanly at zero
        self._got_first_encoder = False

        self.imu_callback_counter = 0
        self.start = time.time()

    def imu_callback(self, msg: Imu):
        self.imu_callback_counter = self.imu_callback_counter + 1
        self.get_logger().info(f"Counter: {self.imu_callback_counter}")
        self.get_logger().info(f"Time [s]: {(time.time()-self.start):.1f}")
        self.get_logger().info(f"Yaw [degrees]: {(self._yaw_imu/math.pi*180):.3f}")
        q = msg.orientation
        # Convert quaternion → Euler
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self._IMU_offset is None:
            self._IMU_offset = yaw

        self._yaw_imu = -yaw + self._IMU_offset - self._start_offset  # type: ignore

    def encoder_callback(self, msg: Encoders):
        # Ignore the first encoder message so we don't start with a jump
        if not self._got_first_encoder:
            self._got_first_encoder = True

            stamp = msg.header.stamp
            self.broadcast_transform(stamp, self._x, self._y, self._yaw)
            self.publish_path(stamp, self._x, self._y, self._yaw)
            return

        # The kinematic parameters for the differential configuration
        ticks_per_rev = 50 * 64
        wheel_radius = 0.04921

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        phi_L = (delta_ticks_left / ticks_per_rev) * 2 * math.pi  # d_phi = K*delta_E
        phi_R = (delta_ticks_right / ticks_per_rev) * 2 * math.pi
        D = wheel_radius / 2 * (phi_R + phi_L)

        self._yaw = self._yaw_imu
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
        self._x += D * math.cos(self._yaw)
        self._y += D * math.sin(self._yaw)

        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self._x, self._y, self._yaw)
        self.publish_path(stamp, self._x, self._y, self._yaw)

    def broadcast_transform(self, stamp, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

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
