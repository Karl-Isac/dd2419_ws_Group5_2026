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
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.red_published = False
        self.red_available = False
        self.red_timestamp = None

        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'camera_color_optical_frame'
        static_tf.transform.translation.x = 0.08987
        static_tf.transform.translation.y = 0.0175
        static_tf.transform.translation.z = 0.10456
        q = quaternion_from_euler(-np.pi/2, 0, -np.pi/2)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(static_tf)


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
        sum_x = 0
        sum_y = 0
        sum_z = 0
        counter = 0

        tf_red = TransformStamped()

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
            if z < 0.9 and y < 0.105:
                if colors[idx, 0] > 0.6 and colors[idx, 1] < 0.4 and colors[idx, 2] < 0.4:
                    counter += 1
                    red_points.append([x,y,z])
                    sum_x += x
                    sum_y += y
                    sum_z += z


                # if colors[idx, 0] < 0.5 and colors[idx, 1] > 0.5 and colors[idx, 2] < 0.5:
                    # self.get_logger().info('Green object detected.')       
           
        
        if counter > 40 and not self.red_available:
            self.get_logger().info('Red object detected.')   
            self.red = tf2_geometry_msgs.PoseStamped()
            self.red.header = msg.header
            self.red.pose.position.x = sum_x / counter
            self.red.pose.position.y = sum_y / counter
            self.red.pose.position.z = sum_z / counter
            self.red.pose.orientation.x = 0.0
            self.red.pose.orientation.y = 0.0
            self.red.pose.orientation.z = 0.0
            self.red.pose.orientation.w = 1.0

            self.red_timestamp = msg.header.stamp
            self.red_available = True

        

        if self.red_available and not self.red_published:
            msg_time = rclpy.time.Time.from_msg(self.red_timestamp)
            if not self.tf_buffer.can_transform(
                'map',
                self.red.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                red_map = self.tf_buffer.transform(
                    self.red,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform red object from '
                    f'{self.red.header.frame_id} to map: {ex}'
                )
                return
            
            tf_red.header.stamp = self.red_timestamp

            tf_red.header.frame_id = 'map'
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