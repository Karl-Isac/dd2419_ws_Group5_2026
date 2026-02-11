#!/usr/bin/env python

import math
import std_msgs.msg

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_geometry_msgs
from tf2_ros import PointStamped, TransformBroadcaster, TransformListener, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/realsense/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/realsense/depth/color/points', self.cloud_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.red_published = False
        self.red_available = False
        self.red_timestamp = None

        self.blue_published = False
        self.blue_available = False
        self.blue_timestamp = None

        self.green_published = False
        self.green_available = False
        self.green_timestamp = None

        self.wood_published = False

        self.wood_available = False
        self.wood_timestamp = None

        # static_tf = TransformStamped()
        # static_tf.header.stamp = self.get_clock().now().to_msg()
        # static_tf.header.frame_id = 'base_link'
        # static_tf.child_frame_id = 'camera_color_optical_frame'
        # static_tf.transform.translation.x = 0.08987
        # static_tf.transform.translation.y = 0.0175
        # static_tf.transform.translation.z = 0.10456
        # q = quaternion_from_euler(-np.pi/2, 0, -np.pi/2)
        # static_tf.transform.rotation.x = q[0]
        # static_tf.transform.rotation.y = q[1]
        # static_tf.transform.rotation.z = q[2]
        # static_tf.transform.rotation.w = q[3]

        # self.static_broadcaster.sendTransform(static_tf)


    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """

        # Convert ROS -> NumPy

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        red_points = []
        red_sum_x = 0
        red_sum_y = 0
        red_sum_z = 0
        red_counter = 0
        tf_red = TransformStamped()

        blue_points = []
        blue_sum_x = 0
        blue_sum_y = 0
        blue_sum_z = 0
        blue_counter = 0
        tf_blue = TransformStamped()

        green_points = []
        green_sum_x = 0
        green_sum_y = 0
        green_sum_z = 0
        green_counter = 0
        tf_green = TransformStamped()

        wood_points = []
        wood_sum_x = 0
        wood_sum_y = 0
        wood_sum_z = 0
        wood_counter = 0
        tf_wood = TransformStamped()

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255

        for idx in range(points.shape[0]):
            x, y, z = points[idx]
            r = colors[idx, 0]
            g = colors[idx, 1]
            b = colors[idx, 2]
            h, s, v = self.rgb_to_hsv(r, g, b)
            if y > 0 and z > 0 and z < 0.5:
                # red
                if (h <= 10 or h >= 160) and s > 0.5 and v > 0.4:
                    red_counter += 1
                    red_points.append([x,y,z])
                    red_sum_x += x
                    red_sum_y += y
                    red_sum_z += z
                # blue
                if (h >= 180 and h <= 240) and s > 0.5 and v > 0.4:
                    blue_counter += 1
                    blue_points.append([x,y,z])
                    blue_sum_x += x
                    blue_sum_y += y
                    blue_sum_z += z
                # green
                if 80 <= h <= 140 and s > 0.5 and v > 0.4:
                    green_counter += 1
                    green_points.append([x,y,z])
                    green_sum_x += x
                    green_sum_y += y
                    green_sum_z += z
                # wood
                if 20 <= h <= 40 and s > 0.3 and v > 0.5:
                    wood_counter += 1
                    wood_points.append([x,y,z])
                    wood_sum_x += x
                    wood_sum_y += y
                    wood_sum_z += z

        # red 
        if red_counter > 15 and not self.red_available:
            self.get_logger().info('Red object detected.')
            self.red = tf2_geometry_msgs.PoseStamped()
            self.red.header = msg.header
            self.red.pose.position.x = red_sum_x / red_counter
            self.red.pose.position.y = red_sum_y / red_counter
            self.red.pose.position.z = red_sum_z / red_counter
            self.red.pose.orientation.x = 0.0
            self.red.pose.orientation.y = 0.0
            self.red.pose.orientation.z = 0.0
            self.red.pose.orientation.w = 1.0

            self.red_timestamp = msg.header.stamp
            self.red_available = True

        if self.red_available and not self.red_published:
            msg_time = rclpy.time.Time.from_msg(self.red_timestamp)
            if not self.tf_buffer.can_transform(
                'realsense_camera_link',
                self.red.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                red_map = self.tf_buffer.transform(
                    self.red,
                    'realsense_camera_link',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform red object from '
                    f'{self.red.header.frame_id} to map: {ex}'
                )
                return
            
            tf_red.header.stamp = self.red_timestamp

            tf_red.header.frame_id = 'realsense_camera_link'
            tf_red.child_frame_id = 'red_object'

            tf_red.transform.translation.x = red_map.pose.position.x
            tf_red.transform.translation.y = red_map.pose.position.y
            tf_red.transform.translation.z = red_map.pose.position.z

            tf_red.transform.rotation.x = 0.0
            tf_red.transform.rotation.y = 0.0
            tf_red.transform.rotation.z = 0.0
            tf_red.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform(tf_red)
            self.red_published = True

        # blue
        if blue_counter > 15 and not self.blue_available:
            self.get_logger().info('Blue object detected.')
            self.get_logger().info(f'coordinates: x={blue_sum_x / blue_counter}, y={blue_sum_y / blue_counter}, z={blue_sum_z / blue_counter}')  
            self.get_logger().info(f'color: r={colors[idx, 0]}, g={colors[idx, 1]}, b={colors[idx, 2]}') 
            self.blue = tf2_geometry_msgs.PoseStamped()
            self.blue.header = msg.header
            self.blue.pose.position.x = blue_sum_x / blue_counter
            self.blue.pose.position.y = blue_sum_y / blue_counter
            self.blue.pose.position.z = blue_sum_z / blue_counter
            self.blue.pose.orientation.x = 0.0
            self.blue.pose.orientation.y = 0.0
            self.blue.pose.orientation.z = 0.0
            self.blue.pose.orientation.w = 1.0

            self.blue_timestamp = msg.header.stamp
            self.blue_available = True

        if self.blue_available and not self.blue_published:
            msg_time = rclpy.time.Time.from_msg(self.blue_timestamp)
            if not self.tf_buffer.can_transform(
                'realsense_camera_link',
                self.blue.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                blue_map = self.tf_buffer.transform(
                    self.blue,
                    'realsense_camera_link',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform blue object from '
                    f'{self.blue.header.frame_id} to map: {ex}'
                )
                return
            
            tf_blue.header.stamp = self.blue_timestamp

            tf_blue.header.frame_id = 'realsense_camera_link'
            tf_blue.child_frame_id = 'blue_object'

            tf_blue.transform.translation.x = blue_map.pose.position.x
            tf_blue.transform.translation.y = blue_map.pose.position.y
            tf_blue.transform.translation.z = blue_map.pose.position.z

            tf_blue.transform.rotation.x = 0.0
            tf_blue.transform.rotation.y = 0.0
            tf_blue.transform.rotation.z = 0.0
            tf_blue.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform(tf_blue)
            self.blue_published = True
        
        # green
        if green_counter > 5 and not self.green_available:
            self.get_logger().info('Green object detected.')   
            self.green = tf2_geometry_msgs.PoseStamped()
            self.green.header = msg.header
            self.green.pose.position.x = green_sum_x / green_counter
            self.green.pose.position.y = green_sum_y / green_counter
            self.green.pose.position.z = green_sum_z / green_counter
            self.green.pose.orientation.x = 0.0
            self.green.pose.orientation.y = 0.0
            self.green.pose.orientation.z = 0.0
            self.green.pose.orientation.w = 1.0

            self.green_timestamp = msg.header.stamp
            self.green_available = True
        
        if self.green_available and not self.green_published:
            msg_time = rclpy.time.Time.from_msg(self.green_timestamp)
            if not self.tf_buffer.can_transform(
                'realsense_camera_link',
                self.green.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                green_map = self.tf_buffer.transform(
                    self.green,
                    'realsense_camera_link',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform green object from '
                    f'{self.green.header.frame_id} to map: {ex}'
                )
                return
            
            tf_green.header.stamp = self.green_timestamp

            tf_green.header.frame_id = 'realsense_camera_link'
            tf_green.child_frame_id = 'green_object'

            tf_green.transform.translation.x = green_map.pose.position.x
            tf_green.transform.translation.y = green_map.pose.position.y
            tf_green.transform.translation.z = green_map.pose.position.z

            tf_green.transform.rotation.x = 0.0
            tf_green.transform.rotation.y = 0.0
            tf_green.transform.rotation.z = 0.0
            tf_green.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform(tf_green)
            self.green_published = True

        # wood
        if wood_counter > 5 and not self.wood_available:
            self.get_logger().info('Wood object detected.')   
            self.wood = tf2_geometry_msgs.PoseStamped()
            self.wood.header = msg.header
            self.wood.pose.position.x = wood_sum_x / wood_counter
            self.wood.pose.position.y = wood_sum_y / wood_counter
            self.wood.pose.position.z = wood_sum_z / wood_counter
            self.wood.pose.orientation.x = 0.0
            self.wood.pose.orientation.y = 0.0
            self.wood.pose.orientation.z = 0.0
            self.wood.pose.orientation.w = 1.0

            self.wood_timestamp = msg.header.stamp
            self.wood_available = True
        
        if self.wood_available and not self.wood_published:
            msg_time = rclpy.time.Time.from_msg(self.wood_timestamp)
            if not self.tf_buffer.can_transform(
                'realsense_camera_link',
                self.wood.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                wood_map = self.tf_buffer.transform(
                    self.wood,
                    'realsense_camera_link',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform wood object from '
                    f'{self.wood.header.frame_id} to map: {ex}'
                )
                return
            
            tf_wood.header.stamp = self.wood_timestamp

            tf_wood.header.frame_id = 'realsense_camera_link'
            tf_wood.child_frame_id = 'wood_object'

            tf_wood.transform.translation.x = wood_map.pose.position.x
            tf_wood.transform.translation.y = wood_map.pose.position.y
            tf_wood.transform.translation.z = wood_map.pose.position.z

            tf_wood.transform.rotation.x = 0.0
            tf_wood.transform.rotation.y = 0.0
            tf_wood.transform.rotation.z = 0.0
            tf_wood.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform(tf_wood)
            self.wood_published = True

        # if self.red_available and not self.red_published:
        #     msg_time = rclpy.time.Time.from_msg(self.red_timestamp)
        #     if not self.tf_buffer.can_transform(
        #         'map',
        #         self.red.header.frame_id,
        #         msg_time,
        #         timeout=rclpy.duration.Duration(seconds=1)
        #     ):
        #         return

        #     try:
        #         red_map = self.tf_buffer.transform(
        #             self.red,
        #             'map',
        #             timeout=rclpy.duration.Duration(seconds=1)
        #         )
        #     except TransformException as ex:
        #         self.get_logger().info(
        #             f'Could not transform red object from '
        #             f'{self.red.header.frame_id} to map: {ex}'
        #         )
        #         return
            
        #     tf_red.header.stamp = self.red_timestamp

        #     tf_red.header.frame_id = 'map'
        #     tf_red.child_frame_id = 'red_object'

        #     tf_red.transform.translation.x = red_map.pose.position.x
        #     tf_red.transform.translation.y = red_map.pose.position.y
        #     tf_red.transform.translation.z = red_map.pose.position.z

        #     tf_red.transform.rotation.x = 0.0
        #     tf_red.transform.rotation.y = 0.0
        #     tf_red.transform.rotation.z = 0.0
        #     tf_red.transform.rotation.w = 1.0

        #     self.static_broadcaster.sendTransform(tf_red)
        #     self.red_published = True
    
    def rgb_to_hsv(r, g, b):
        c_max = max(r, g, b)
        c_min = min(r, g, b)
        delta = c_max - c_min

        if delta == 0:
            h = 0.0
        elif c_max == r:
            h = 60.0 * (((g - b) / delta) % 6)
        elif c_max == g:
            h = 60.0 * (((b - r) / delta) + 2)
        elif c_max == b:
            h = 60.0 * (((r - g) / delta) + 4)

        s = 0.0 if c_max == 0 else delta / c_max

        v = c_max

        return h, s, v
        
 
def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()