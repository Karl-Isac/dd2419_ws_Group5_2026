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
from geometry_msgs.msg import TransformStamped, Point, Vector3Stamped, PoseArray, Pose
from visualization_msgs.msg import Marker

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import csv
from ament_index_python.packages import get_package_share_directory
import os

import ctypes
import struct

# Criteria of colors are at Line 468-478

######################################################################################################
# TODO: discuss the unit of the communication (PoseArray): m
######################################################################################################

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

        # initialize topic publisher
        self.objects_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        self.boxes_pub = self.create_publisher(PoseArray, '/detected_boxes', 10)

        # open and load map file (csv)
        package_path = get_package_share_directory('detection')
        # csv_path = os.path.join(package_path, 'config', 'test.csv')
        # csv_path = os.path.join(package_path, 'config', 'map_1_1.csv')
        csv_path = os.path.join(package_path, 'config', 'map_1_2.csv')
        self.metadata_rows = []

        # location of final map file (csv)
        self.output_csv_path = os.path.join(
            os.path.expanduser('~/dd2419_ws_Group5_2026/src/detection/config/'),
            'detection_output.csv'
        )
        os.makedirs(os.path.dirname(self.output_csv_path), exist_ok=True)
        self.get_logger().info(f'Output CSV will be written to {self.output_csv_path}')

        self.object_poses = []
        self.box_poses = []
        self.object_lists = []
        self.box_lists = []

        self.object_num = 0
        self.box_num = 0

        with open(csv_path, mode='r', encoding='utf-8') as file:
            reader = csv.reader(file)
            # skip the first line (header)
            header = next(reader)
            for row in reader:
                type_id = row[0]
                x = int(row[1])
                y = int(row[2])
                angle_deg = float(row[3])   
                angle_rad = math.radians(angle_deg)

                pose = Pose()
                pose.position.x = x / 100.0  # convert cm to m
                pose.position.y = y / 100.0  # convert cm to m
                pose.position.z = 0.0        # z is 0
                q = quaternion_from_euler(0, 0, angle_rad)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                if type_id == 'O':
                    self.object_poses.append(pose)
                    self.object_lists.append([x, y, angle_deg])
                elif type_id == 'B':
                    self.box_poses.append(pose)
                    self.box_lists.append([x, y, angle_deg])
                else:
                    self.metadata_rows.append(row)

        self.publish_arrays(self.object_poses, None, self.box_poses, None)
        # print(self.object_lists)
    
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'realsense_camera_link'
        static_tf.transform.translation.x = 0.08987
        static_tf.transform.translation.y = 0.0175
        static_tf.transform.translation.z = 0.10456
        q = quaternion_from_euler(0, 0, 0)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(static_tf)

    def publish_arrays(self, object_poses, object_timestamp, box_poses, box_timestamp):
        """publish object and box poses from map file to ROS topics."""
        # objects
        if object_poses is not None:
            obj_msg = PoseArray()
            obj_msg.header.stamp = object_timestamp if object_timestamp is not None else self.get_clock().now().to_msg()
            obj_msg.header.frame_id = 'map'      
            obj_msg.poses = object_poses

            self.objects_pub.publish(obj_msg)

        # boxes
        if box_poses is not None:
            box_msg = PoseArray()
            box_msg.header.stamp = box_timestamp if box_timestamp is not None else self.get_clock().now().to_msg()
            box_msg.header.frame_id = 'map'
            box_msg.poses = box_poses
            self.boxes_pub.publish(box_msg)

        # self.get_logger().info(f'Published {len(object_poses)} objects and {len(box_poses)} boxes')

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

        blue_points = []
        blue_sum_x = 0
        blue_sum_y = 0
        blue_sum_z = 0
        blue_counter = 0

        green_points = []
        green_sum_x = 0
        green_sum_y = 0
        green_sum_z = 0
        green_counter = 0

        wood_points = []
        wood_sum_x = 0
        wood_sum_y = 0
        wood_sum_z = 0
        wood_counter = 0

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
            if y > 0 and y < 0.09 and z > 0 and z < 1.5:
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
                    grey_points.append([z ,-x])

        # red 
        if red_counter > 10:
            self.object_detection(msg, red_sum_x, red_sum_y, red_sum_z, red_counter, 'Red')

        # blue
        if blue_counter > 10:
            self.object_detection(msg, blue_sum_x, blue_sum_y, blue_sum_z, blue_counter, 'Blue')
        
        # green
        if green_counter > 10:
            self.object_detection(msg, green_sum_x, green_sum_y, green_sum_z, green_counter, 'Green')

        # wood
        if wood_counter > 10:
            self.object_detection(msg, wood_sum_x, wood_sum_y, wood_sum_z, wood_counter, 'Wood')
            

        self.publish_2d_cloud(grey_points, msg.header)

        box_size = (0.24, 0.16)  # L, W

        center, yaw, axes = self.estimate_box_from_points(grey_points, box_size)
        if center is not None:
            # --- convert to map frame ---
            try:
                point_camera = PointStamped()
                point_camera.header.frame_id = 'realsense_camera_link'
                point_camera.header.stamp = rclpy.time.Time().to_msg()
                point_camera.point.x = float(center[0])
                point_camera.point.y = float(center[1])
                point_camera.point.z = 0.0
                point_map = self.tf_buffer.transform(point_camera, 'map')

                dir_camera = Vector3Stamped()
                dir_camera.header.frame_id = 'realsense_camera_link'
                dir_camera.header.stamp = rclpy.time.Time().to_msg()
                dir_camera.vector.x = np.cos(yaw)
                dir_camera.vector.y = np.sin(yaw)
                dir_camera.vector.z = 0.0
                dir_map = self.tf_buffer.transform(dir_camera, 'map')
                map_yaw = np.arctan2(dir_map.vector.y, dir_map.vector.x)

                map_yaw_deg = np.degrees(map_yaw)
                map_yaw_deg = map_yaw_deg % 180

                angle_int = int(round(map_yaw_deg)) % 180

                x_str = int(round(point_map.point.x * 100))
                y_str = int(round(point_map.point.y * 100))

                self.get_logger().info(f'Map box: B {x_str} {y_str} {angle_int}')

                # publish static TF for the box
                tf_map_box = TransformStamped()
                tf_map_box.header.stamp = rclpy.time.Time().to_msg()
                tf_map_box.header.frame_id = 'map'
                tf_map_box.child_frame_id = 'grey_box_map'
                tf_map_box.transform.translation.x = point_map.point.x
                tf_map_box.transform.translation.y = point_map.point.y
                tf_map_box.transform.translation.z = 0
                q = quaternion_from_euler(0.0, 0.0, angle_int * np.pi / 180)
                tf_map_box.transform.rotation.x = q[0]
                tf_map_box.transform.rotation.y = q[1]
                tf_map_box.transform.rotation.z = q[2]
                tf_map_box.transform.rotation.w = q[3]
                self.static_broadcaster.sendTransform(tf_map_box)

                for item in self.box_lists:
                    if np.abs(item[0] - x_str) < 10 and np.abs(item[1] - y_str) < 10:
                        self.get_logger().info("repeated box detection, discarded")
                        break
                else:
                    self.box_lists.append([x_str, y_str, angle_int])
                    new_box_msg = Pose()
                    new_box_msg.position.x = point_map.point.x
                    new_box_msg.position.y = point_map.point.y
                    new_box_msg.position.z = 0.0
                    new_box_msg.orientation.x = tf_map_box.transform.rotation.x
                    new_box_msg.orientation.y = tf_map_box.transform.rotation.y
                    new_box_msg.orientation.z = tf_map_box.transform.rotation.z
                    new_box_msg.orientation.w = tf_map_box.transform.rotation.w
                    self.publish_arrays(None, None, [new_box_msg], rclpy.time.Time().to_msg())

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
    
    def object_detection(self, msg, sum_x, sum_y, sum_z, counter, color):
        # object_num is the number of detected objects, regardless of color, used for TF frame naming
        self.get_logger().info(f'{color} object detected.')
        self.object = tf2_geometry_msgs.PoseStamped()
        self.object.header = msg.header
        self.object.header.stamp = rclpy.time.Time().to_msg()
        self.object.pose.position.x = sum_x / counter
        self.object.pose.position.y = sum_y / counter
        self.object.pose.position.z = sum_z / counter
        self.object.pose.orientation.x = 0.0
        self.object.pose.orientation.y = 0.0
        self.object.pose.orientation.z = 0.0
        self.object.pose.orientation.w = 1.0

        msg_time = rclpy.time.Time().to_msg()
        if not self.tf_buffer.can_transform(
                'map',
                self.object.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                self.get_logger().warn(f'Failed to publish {color} object_{self.object_num}')

        try:
                object_map = self.tf_buffer.transform(
                    self.object,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
        except TransformException as ex:
            self.get_logger().info(
                    f'Could not transform {color} object from '
                    f'{self.object.header.frame_id} to map: {ex}'
            )
            return
        
        tf = TransformStamped()
        tf.header.stamp = rclpy.time.Time().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = f'object_{color}'
        tf.transform.translation.x = object_map.pose.position.x
        tf.transform.translation.y = object_map.pose.position.y
        tf.transform.translation.z = object_map.pose.position.z
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

        self.get_logger().info(f'Map box: {color} {object_map.pose.position.x} {object_map.pose.position.y} N/A')

        for item in self.object_lists:
            if np.abs(item[0] - object_map.pose.position.x * 100) < 3 and np.abs(item[1] - object_map.pose.position.y * 100) < 3:
                self.get_logger().info(f"repeated {color} detection, discarded")
                break
        else:
            self.object_lists.append([int(round(object_map.pose.position.x * 100)), int(round(object_map.pose.position.y * 100)), 0])
            new_object_msg = Pose()
            new_object_msg.position.x = object_map.pose.position.x
            new_object_msg.position.y = object_map.pose.position.y
            new_object_msg.position.z = 0.0
            new_object_msg.orientation.x = 0.0
            new_object_msg.orientation.y = 0.0
            new_object_msg.orientation.z = 0.0
            new_object_msg.orientation.w = 1.0
            self.publish_arrays([new_object_msg], rclpy.time.Time().to_msg(), None, None)
            self.object_num += 1

        
    def publish_2d_cloud(self, points_xz, header):
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
        
        if len(points) < 10: 
            return None, None, None 
        
        pts = np.array(points) 
        
        # --- Step 0: reduce outliers（IQR method） --- 
        
        Q1 = np.percentile(pts, 25, axis=0) 
        Q3 = np.percentile(pts, 75, axis=0) 
        IQR = Q3 - Q1 
        mask = np.all((pts >= Q1 - 1.5 * IQR) & (pts <= Q3 + 1.5 * IQR), axis=1) 
        pts = pts[mask] 
        
        if len(pts) < 2: 
            return None, None, None 
        
        # --- Step 1: PCA --- 
        mean = np.mean(pts, axis=0) 
        pts_centered = pts - mean 
        U, S, Vt = np.linalg.svd(pts_centered, full_matrices=False) 
        axes = Vt[:2] 
        
        # --- Step 2: angle calculation --- 
        dir1 = axes[0] 
        dir2 = axes[1] 
        dir1 /= np.linalg.norm(dir1) 
        dir2 /= np.linalg.norm(dir2) 
        dir1 = dir1 if dir1[1] >= 0 else -dir1 # y > 0
        dir2 = dir2 if dir2[1] >= 0 else -dir2 # y > 0
        
        # angle with respect to x-axis
        x_axis = np.array([1.0, 0.0]) 
        angle_dir1_x = np.arccos(np.clip(np.dot(dir1, x_axis), -1.0, 1.0)) 
        angle_dir2_x = np.arccos(np.clip(np.dot(dir2, x_axis), -1.0, 1.0)) 
        
        # Step 3: two edge vs single edge decision based on variance ratio
        
        ratio = S[1] / S[0] 
        self.get_logger().debug(f'variance ratio: {ratio:.3f}') 

        if ratio > 0.1:
            # =========================================================
            # RANSAC - two edges
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

            # first edge
            model1, inliers1 = fit_line_ransac(pts_np)

            if model1 is None or len(inliers1) < 5:
                return None, None, None

            mask = np.ones(len(pts_np), dtype=bool)
            for p in inliers1:
                idx = np.where((pts_np == p).all(axis=1))[0]
                mask[idx] = False

            remaining = pts_np[mask]

            # second edge
            model2, inliers2 = fit_line_ransac(remaining)

            if model2 is None or len(inliers2) < 5:
                return None, None, None

            # corner point
            corner = intersect_lines(model1, model2)

            # =========================================================
            # direction vectors and used axes
            # =========================================================
            n1, _ = model1
            n2, _ = model2

            dir1 = np.array([n1[1], -n1[0]])
            dir2 = np.array([n2[1], -n2[0]])

            dir1 /= np.linalg.norm(dir1)
            dir2 /= np.linalg.norm(dir2)

            # keeps x > 0 in camera frame
            if dir1[1] < 0:
                dir1 = -dir1
            if dir2[1] < 0:
                dir2 = -dir2

            used_axes = np.vstack([dir1, dir2])

            # =========================================================
            # length estimation along each direction
            # =========================================================
            proj1 = pts_np @ dir1
            proj2 = pts_np @ dir2

            length1 = proj1.max() - proj1.min()
            length2 = proj2.max() - proj2.min()

            self.get_logger().debug(
                f'RANSAC length1: {length1:.3f}, length2: {length2:.3f}'
            )

            # judge which direction corresponds to length vs width based on variance and box size ratio
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
            # calculate center by shifting from corner along main_dir and side_dir
            # =========================================================
            # center_shifted = corner + main_dir * (box_length / 2)
            # center_shifted = corner - main_dir * (box_length / 2) + side_dir * (box_width / 2) # shift from corner along both directions to get to the center, more robust for partial views
    
            center_shifted = corner
            center_shifted = center_shifted + main_dir * (box_length / 2) if main_dir[0] > 0 else center_shifted - main_dir * (box_length / 2)
            center_shifted = center_shifted + side_dir * (box_width / 2) if side_dir[0] > 0 else center_shifted - side_dir * (box_width / 2)
            # print(f"main_dir is {main_dir}, side_dir is {side_dir}")

            # yaw
            yaw = np.arctan2(main_dir[1], main_dir[0])

            self.get_logger().debug(
                f'Corner: {corner}, Center: {center_shifted}, yaw: {yaw:.3f}'
            )

            return center_shifted, yaw, used_axes

        else: 
            # =========================================================
            # single edge case - use PCA axes, shift center along normal direction to get to box
            # =========================================================
            
            used_axes = axes[:1] # only use the first principal axis if it's not a corner 
            normal = axes[1] if np.dot(axes[1], x_axis) > 0 else -axes[1] 
            projected = pts_centered @ used_axes.T 

            self.get_logger().debug(f'dir1 与 x 轴夹角: {angle_dir1_x:.2f}rad, dir2 与 x 轴夹角: {angle_dir2_x:.2f}rad') 
            min_proj = projected.min(axis=0) 
            max_proj = projected.max(axis=0) 
            center_proj = (min_proj + max_proj) / 2 
            center = mean + center_proj @ used_axes # 
            length_proj = projected[:,0].max() - projected[:,0].min()
            width_proj = length_proj # set width same as length for single edge case, will be corrected by shifting and box size later

            self.get_logger().debug(f'length_proj: {length_proj:.3f}, width_proj: {width_proj:.3f}') 

            if length_proj >= width_proj: 
                # length corresponds to first principal axis → keep order
                box_length = box_size[0] 
                box_width = box_size[1] 
            else: 
                # length corresponds to second principal axis → swap order
                used_axes = used_axes[::-1] 
                box_length = box_size[1] 
                box_width = box_size[0] 
            
            if length_proj >= box_width: 
                shift_vec = normal * (box_width / 2) # shift along normal direction to get to center
                yaw = angle_dir1_x 
            else: 
                shift_vec = normal * (box_length / 2) # shift along normal direction to get to center
                yaw = angle_dir1_x - np.pi/2 if angle_dir1_x < np.pi/2 - 0.01 else angle_dir1_x - np.pi/2 

            center_shifted = center + shift_vec 

            return center_shifted, yaw, used_axes
    
    def write_csv(self):
        try:
            with open(self.output_csv_path, mode='w', encoding='utf-8', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Type', 'x', 'y', 'angle'])
                for meta_row in self.metadata_rows:
                    writer.writerow(meta_row)
                for obj in self.object_lists:
                    writer.writerow(['O'] + obj)
                for box in self.box_lists:
                    writer.writerow(['B'] + box)
            self.get_logger().debug(f'CSV file updated: {self.output_csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write CSV: {e}')

 
def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down, writing CSV...')
    finally:
        node.write_csv()
        node.destroy_node()
        rclpy.shutdown()

def is_red(h,s,v):
    return True if (h <= 20 or h >= 340) and s > 0.8 and v > 0.5 else False

def is_blue(h,s,v):
    return True if (h >= 180 and h <= 200) and s > 0.8 and v > 0.4 else False

def is_green(h,s,v):
    return True if 140 <= h <= 180 and s > 0.8 and v > 0.25 else False

def is_wood(h,s,v):
    return True if 20 <= h <= 60 and 0.3 < s < 0.6 and 0.3 < v < 0.5 else False

def is_grey(h,s,v):
    return True if 0.05 < s < 0.15 and v > 0.15 and v < 0.3 else False

if __name__ == '__main__':
    main()
