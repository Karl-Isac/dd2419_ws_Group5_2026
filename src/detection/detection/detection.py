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
from geometry_msgs.msg import TransformStamped, Point, Vector3Stamped
from visualization_msgs.msg import Marker

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


import ctypes
import struct

# Criteria of colors are at Line 468-478

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

        grey_points = []

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
                if is_red(h, s, v):
                    red_counter += 1
                    red_points.append([x,y,z])
                    red_sum_x += x
                    red_sum_y += y
                    red_sum_z += z
                # blue
                elif is_blue(h,s,v):
                    blue_counter += 1
                    blue_points.append([x,y,z])
                    blue_sum_x += x
                    blue_sum_y += y
                    blue_sum_z += z
                # green
                elif is_green(h,s,v):
                    green_counter += 1
                    green_points.append([x,y,z])
                    green_sum_x += x
                    green_sum_y += y
                    green_sum_z += z
                # wood
                elif is_wood(h,s,v):
                    wood_counter += 1
                    wood_points.append([x,y,z])
                    wood_sum_x += x
                    wood_sum_y += y
                    wood_sum_z += z

                if is_grey(h,s,v):
                    grey_points.append([z ,-x])

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

            self.get_logger().info(f'Map box: Red {red_map.pose.position.x} {red_map.pose.position.y} N/A')

        # blue
        if blue_counter > 15 and not self.blue_available:
            self.get_logger().info('Blue object detected.')
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
                'map',
                self.blue.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                blue_map = self.tf_buffer.transform(
                    self.blue,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform blue object from '
                    f'{self.blue.header.frame_id} to map: {ex}'
                )
                return
            
            tf_blue.header.stamp = self.blue_timestamp

            tf_blue.header.frame_id = 'map'
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

            self.get_logger().info(f'Map box: Blue {blue_map.pose.position.x} {blue_map.pose.position.y} N/A')
        
        # green
        if green_counter > 15 and not self.green_available:
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
                'map',
                self.green.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                green_map = self.tf_buffer.transform(
                    self.green,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform green object from '
                    f'{self.green.header.frame_id} to map: {ex}'
                )
                return
            
            tf_green.header.stamp = self.green_timestamp

            tf_green.header.frame_id = 'map'
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

            self.get_logger().info(f'Map box: Green {green_map.pose.position.x} {green_map.pose.position.y} N/A')

        # wood
        if wood_counter > 15 and not self.wood_available:
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
                'map',
                self.wood.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                return

            try:
                wood_map = self.tf_buffer.transform(
                    self.wood,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform wood object from '
                    f'{self.wood.header.frame_id} to map: {ex}'
                )
                return
            
            tf_wood.header.stamp = self.wood_timestamp

            tf_wood.header.frame_id = 'map'
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

            self.get_logger().info(f'Map box: Wood {wood_map.pose.position.x} {wood_map.pose.position.y} N/A')

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
            
        #     self.get_logger().info(f'Map box: Red {red_map.pose.position.x} {red_map.pose.position.y} N/A')
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

        self.publish_2d_cloud(grey_points, msg.header)

        box_size = (0.24, 0.16)  # L, W
        
        # center, yaw, axes = self.estimate_box_from_points(grey_points, box_size)

        # if center is not None:
        #     tf_grey = TransformStamped()
        #     tf_grey.header.stamp = msg.header.stamp
        #     tf_grey.header.frame_id = 'realsense_camera_link'
        #     tf_grey.child_frame_id = 'grey_box'

        #     # 位置
        #     tf_grey.transform.translation.x = float(center[0])
        #     tf_grey.transform.translation.y = float(center[1])
        #     tf_grey.transform.translation.z = 0.05  # 高度固定为点云平面上方一点

        #     # 旋转（绕 Z 轴 yaw）
        #     q = quaternion_from_euler(0.0, 0.0, float(yaw))
        #     tf_grey.transform.rotation.x = q[0]
        #     tf_grey.transform.rotation.y = q[1]
        #     tf_grey.transform.rotation.z = q[2]
        #     tf_grey.transform.rotation.w = q[3]

        #     self.static_broadcaster.sendTransform(tf_grey)

        # 在 cloud_callback 中 estimate_box_from_points 之后
        center, yaw, axes = self.estimate_box_from_points(grey_points, box_size)
        if center is not None:
            # --- 转换到 map 坐标系 ---
            try:
                # # 获取变换
                # transform = self.tf_buffer.lookup_transform(
                #     'map', 'realsense_camera_link', msg.header.stamp, timeout=rclpy.duration.Duration(seconds=0.5))
                
                # 转换位置
                point_camera = PointStamped()
                point_camera.header.frame_id = 'realsense_camera_link'
                point_camera.header.stamp = msg.header.stamp
                point_camera.point.x = float(center[0])
                point_camera.point.y = float(center[1])
                point_camera.point.z = 0.0
                point_map = self.tf_buffer.transform(point_camera, 'map')

                # 转换方向
                dir_camera = Vector3Stamped()
                dir_camera.header.frame_id = 'realsense_camera_link'
                dir_camera.header.stamp = msg.header.stamp
                dir_camera.vector.x = np.cos(yaw)
                dir_camera.vector.y = np.sin(yaw)
                dir_camera.vector.z = 0.0
                dir_map = self.tf_buffer.transform(dir_camera, 'map')
                map_yaw = np.arctan2(dir_map.vector.y, dir_map.vector.x)

                # 将 map_yaw 从弧度转换为度
                map_yaw_deg = np.degrees(map_yaw)

                # 归一化到 [0, 180) 范围（取模 180）
                map_yaw_deg = map_yaw_deg % 180

                # 四舍五入取整，并确保在 0~179 之间（取模 180 后自动在 [0,180)，但可能刚好 180 变成 0）
                angle_int = int(round(map_yaw_deg)) % 180

                # 格式化 x, y 保留两位小数
                x_str = f"{point_map.point.x*100:.2f}"
                y_str = f"{point_map.point.y*100:.2f}"

                # 现在你可以将 (x_str, y_str, angle_int) 写入地图文件
                self.get_logger().info(f'Map box: B {x_str} {y_str} {angle_int}')
                
                # 可选：发布一个静态 TF 到 map 下
                tf_map_box = TransformStamped()
                tf_map_box.header.stamp = msg.header.stamp
                tf_map_box.header.frame_id = 'map'
                tf_map_box.child_frame_id = 'grey_box_map'
                tf_map_box.transform.translation.x = point_map.point.x
                tf_map_box.transform.translation.y = point_map.point.y
                tf_map_box.transform.translation.z = 0.05
                q = quaternion_from_euler(0.0, 0.0, angle_int * np.pi / 180)
                tf_map_box.transform.rotation.x = q[0]
                tf_map_box.transform.rotation.y = q[1]
                tf_map_box.transform.rotation.z = q[2]
                tf_map_box.transform.rotation.w = q[3]
                self.static_broadcaster.sendTransform(tf_map_box)

            except TransformException as ex:
                self.get_logger().error(f'Transform failed: {ex}')

    def rgb_to_hsv(self, r, g, b):
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
        
    def publish_2d_cloud(self, points_xz, header):
        # 新header
        h = std_msgs.msg.Header()
        h.stamp = header.stamp
        h.frame_id = 'realsense_camera_link'

        # 2D → 3D
        pts = [(p[0], p[1], 0.05) for p in points_xz]

        cloud = pc2.create_cloud_xyz32(h, pts)
        self._pub.publish(cloud)

    def estimate_box_from_points(self, points, box_size=(0.24, 0.16), angle_thresh_deg=20): 
        " Estimate box position and yaw from a set of 2D points (x, y) on the ground. "
        "points: Nx2 array, points in x-y plane "
        "box_size: (length, width) in meters "
        "angle_thresh_deg: allowable deviation from 90deg to consider a corner "
        
        "Returns: "
        "center_shifted: (x, y) box center shifted along another axis "
        "yaw: rotation around z in radians "
        "axes: principal axes vectors (2x2) """ 
        
        if len(points) < 2: 
            return None, None, None # 不够点无法估计 
        
        pts = np.array(points) 
        
        # --- Step 0: 去除离群点（IQR法） --- 
        
        # Q1 = np.percentile(pts, 25, axis=0) 
        # Q3 = np.percentile(pts, 75, axis=0) 
        # IQR = Q3 - Q1 
        # mask = np.all((pts >= Q1 - 1.5 * IQR) & (pts <= Q3 + 1.5 * IQR), axis=1) 
        # pts = pts[mask] 
        
        if len(pts) < 2: 
            return None, None, None 
        
        # --- Step 1: PCA --- 
        mean = np.mean(pts, axis=0) 
        pts_centered = pts - mean 
        U, S, Vt = np.linalg.svd(pts_centered, full_matrices=False) 
        axes = Vt[:2] # 两个主轴 
        
        # --- Step 2: 判断角度 --- 
        dir1 = axes[0] 
        dir2 = axes[1] 
        dir1 /= np.linalg.norm(dir1) 
        dir2 /= np.linalg.norm(dir2) 
        dir1 = dir1 if dir1[1] >= 0 else -dir1 # 保持第一主轴朝上 
        dir2 = dir2 if dir2[1] >= 0 else -dir2 # 保持第二主轴朝上 
        
        # 计算 dir1 和 dir2 关于 x 轴的夹角 
        x_axis = np.array([1.0, 0.0]) 
        angle_dir1_x = np.arccos(np.clip(np.dot(dir1, x_axis), -1.0, 1.0)) 
        angle_dir2_x = np.arccos(np.clip(np.dot(dir2, x_axis), -1.0, 1.0)) 
        
        # Step 3: 判断是否是角 
        
        ratio = S[1] / S[0] 
        self.get_logger().info(f'主成分方差比: {ratio:.3f}') 

        if ratio > 0.1:
            # =========================================================
            # RANSAC 拟合两条边 → 求角点
            # =========================================================

            pts_np = pts.copy()

            def fit_line_ransac(points, threshold=0.008, max_iter=200):
                best_inliers = []
                best_model = None

                if len(points) < 2:
                    return None, []

                for _ in range(max_iter):
                    i1, i2 = np.random.choice(len(points), 2, replace=False)
                    p1, p2 = points[i1], points[i2]

                    dir_vec = p2 - p1
                    norm = np.linalg.norm(dir_vec)
                    if norm < 1e-6:
                        continue
                    dir_vec /= norm

                    normal = np.array([-dir_vec[1], dir_vec[0]])
                    d = -np.dot(normal, p1)

                    dist = np.abs(points @ normal + d)
                    inliers = points[dist < threshold]

                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
                        best_model = (normal, d)

                return best_model, best_inliers

            def intersect_lines(model1, model2):
                n1, d1 = model1
                n2, d2 = model2
                A = np.vstack([n1, n2])
                b = -np.array([d1, d2])
                return np.linalg.solve(A, b)

            # 第一条边
            model1, inliers1 = fit_line_ransac(pts_np)

            if model1 is None or len(inliers1) < 5:
                return None, None, None

            # 删除第一条边点
            mask = np.ones(len(pts_np), dtype=bool)
            for p in inliers1:
                idx = np.where((pts_np == p).all(axis=1))[0]
                mask[idx] = False

            remaining = pts_np[mask]

            # 第二条边
            model2, inliers2 = fit_line_ransac(remaining)

            if model2 is None or len(inliers2) < 5:
                return None, None, None

            # 角点
            corner = intersect_lines(model1, model2)

            # =========================================================
            # 方向向量（从直线法向恢复）
            # =========================================================
            n1, _ = model1
            n2, _ = model2

            dir1 = np.array([n1[1], -n1[0]])
            dir2 = np.array([n2[1], -n2[0]])

            dir1 /= np.linalg.norm(dir1)
            dir2 /= np.linalg.norm(dir2)

            # 保持朝前
            if dir1[1] < 0:
                dir1 = -dir1
            if dir2[1] < 0:
                dir2 = -dir2

            used_axes = np.vstack([dir1, dir2])

            # =========================================================
            # 计算长度方向
            # =========================================================
            proj1 = pts_np @ dir1
            proj2 = pts_np @ dir2

            length1 = proj1.max() - proj1.min()
            length2 = proj2.max() - proj2.min()

            self.get_logger().info(
                f'RANSAC length1: {length1:.3f}, length2: {length2:.3f}'
            )

            # 判断哪条是长边
            if length1 > length2:
                main_dir = dir1
                side_dir = dir2
                box_length = box_size[0]
                box_width = box_size[1]
            else:
                main_dir = dir2
                side_dir = dir1
                box_length = box_size[0]
                box_width = box_size[1]

            # =========================================================
            # 计算中心
            # =========================================================
            # center_shifted = corner + main_dir * (box_length / 2)
            center_shifted = corner - main_dir * (box_length / 2) + side_dir * (box_width / 2) # 沿宽度方向平移到箱子中心
            # center_shifted = corner

            # yaw
            yaw = np.arctan2(main_dir[1], main_dir[0])

            self.get_logger().info(
                f'Corner: {corner}, Center: {center_shifted}, yaw: {yaw:.3f}'
            )

            return center_shifted, yaw, used_axes

        else: 
            used_axes = axes[:1] # 单边 
            normal = axes[1] if np.dot(axes[1], x_axis) > 0 else -axes[1] 
            is_corner = False 
            projected = pts_centered @ used_axes.T 

            self.get_logger().info(f'dir1 与 x 轴夹角: {angle_dir1_x:.2f}rad, dir2 与 x 轴夹角: {angle_dir2_x:.2f}rad') 
            min_proj = projected.min(axis=0) 
            max_proj = projected.max(axis=0) 
            center_proj = (min_proj + max_proj) / 2 
            center = mean + center_proj @ used_axes # 回到原坐标系 
            length_proj = projected[:,0].max() - projected[:,0].min()
            width_proj = length_proj # 如果不是角，则将宽度设为长度

            self.get_logger().info(f'length_proj: {length_proj:.3f}, width_proj: {width_proj:.3f}') 

            if length_proj >= width_proj: 
                # 第一主轴对应长度 → 第二主轴对应宽度 
                box_length = box_size[0] 
                box_width = box_size[1] 
            else: 
                # 第一主轴对应宽度 → 交换主轴 
                used_axes = used_axes[::-1] 
                box_length = box_size[1] 
                box_width = box_size[0] 
            
            if length_proj >= box_width: 
                shift_vec = normal * (box_width / 2) # 沿宽度方向平移 
                yaw = angle_dir1_x 
            else: 
                shift_vec = normal * (box_length / 2) # 沿长度方向平移 
                yaw = angle_dir1_x - np.pi/2 if angle_dir1_x < np.pi/2 - 0.01 else angle_dir1_x - np.pi/2 

            center_shifted = center + shift_vec 

            return center_shifted, yaw, used_axes

 
def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

def is_red(h,s,v):
    return True if (h <= 20 or h >= 340) and s > 0.5 and v > 0.4 else False

def is_blue(h,s,v):
    return True if (h >= 200 and h <= 260) and s > 0.55 and v > 0.45 else False

def is_green(h,s,v):
    return True if 120 <= h <= 190 and s > 0.4 and v > 0.4 else False

def is_wood(h,s,v):
    return True if 20 <= h <= 60 and 0 < s < 0.6 and v > 0.4 else False

def is_grey(h,s,v):
    return True if s < 0.15 and v > 0.1 and v < 0.25 else False

if __name__ == '__main__':
    main()