#!/usr/bin/env python

import math
import std_msgs.msg

import numpy as np
from collections import deque
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.time import Time
from rclpy.node import Node

import tf2_geometry_msgs
from tf2_ros import PointStamped, TransformBroadcaster, TransformListener, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Point, Vector3Stamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import csv
from ament_index_python.packages import get_package_share_directory
import os

import ctypes
import struct

np.random.seed(42)  # for reproducibility

####################  /home/grumpy/dd2419_ws_Group5_2026/.pixi/envs/default/share/orocos_kdl/cmake/orocos_kdl-config.cmake:13 (include)
##################################################################################
# TODO: discuss the unit of the communication (PoseArray): m
# TODO: One edge situation for box detection is neglected for now
######################################################################################################

class Detection(Node):

    def __init__(self):
        super().__init__('detection')
        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/realsense/depth/color/ds_points', 10)
        
        # TODO: (Private Test) Test the belief range of point cloud of realsense, initialization
        self.test_pub_box = self.create_publisher(
            PointCloud2, '/test_points_box', 10
        )
        self.test_pub_cube = self.create_publisher(
            PointCloud2, '/test_points_cube', 10
        )

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/realsense/depth/color/points', self.cloud_callback, 10)
        
        # TODO: Topic names need to be decided
        self.create_subscription(
            Point, '/Failure', self.redetection_callback, 10)
        self.create_subscription(
            Point, '/Success', self.success_callback, 10
        )
                
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # initialize topic publisher
        self.objects_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        self.boxes_pub = self.create_publisher(PoseArray, '/detected_boxes', 10)

        # === RANSAC VISUALIZATION START ===
        # Publisher for visualizing RANSAC lines, inliers, corner, and center
        self.ransac_viz_pub = self.create_publisher(MarkerArray, '/ransac_visualization', 10)
        # === RANSAC VISUALIZATION END ===

        # open and load map file and workspace (csv)
        package_path = get_package_share_directory('detection')
        map_path = os.path.join(package_path, 'config', 'map_1_5.csv')
        workspace_path = os.path.join(package_path, 'config', 'workspace_1.csv')
        self.metadata_rows = []
        self.boundary = [] # List of intersection of edges of workspace, in format of [[x1, y1], [x2, y2], ...] 

        # location of final map file (csv)
        self.output_map_path = os.path.join(
            os.path.expanduser('~/dd2419_ws_Group5_2026/src/detection/config/'),
            'detection_output.csv'
        )
        os.makedirs(os.path.dirname(self.output_map_path), exist_ok=True)
        self.get_logger().info(f'Output CSV will be written to {self.output_map_path}')

        self.object_poses = []
        self.box_poses = []
        self.object_lists = [] # object_list now stores [x, y, color, status 1, status 2, map_x, map_y], where x and y are position in cm, status 1 tells if the object exists at current position, and status 2 tells whether an object is accepted and no need to redetect, map_x and map_y are coordinates that needs to be written into map file.
        self.box_lists = []

        self.color_unassigned_indices_list = [] # Indices of objects read from map file

        self.object_num = 0
        self.known_obj_num = 0
        self.box_num = 0
        self.known_box_num = 0

        # Using a deque as a buffer to store incoming point cloud messages for processing
        self.cloud_queue = deque(maxlen=10)
        self.timer = self.create_timer(0.1, self.process_queue)

        # Reading map file
        with open(map_path, mode='r', encoding='utf-8') as file:
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
                    self.color_unassigned_indices_list.append(self.object_num)

                    self.object_poses.append(pose)
                    self.object_lists.append([x, y, 'unknown', True, True, x, y]) # color is 'unknown',status 1 is True which means there is an actual object at position now, status 2 is True, which means don't need to be redetected.
                    self.object_num += 1
                    self.known_obj_num += 1
                    
                    tf = TransformStamped()
                    tf.header.stamp = self.get_clock().now().to_msg()
                    tf.header.frame_id = 'map'
                    tf.child_frame_id = f'object_{self.object_num}'
                    tf.transform.translation.x = pose.position.x
                    tf.transform.translation.y = pose.position.y
                    tf.transform.translation.z = pose.position.z
                    tf.transform.rotation.x = 0.0
                    tf.transform.rotation.y = 0.0
                    tf.transform.rotation.z = 0.0
                    tf.transform.rotation.w = 1.0
                    self.static_broadcaster.sendTransform(tf)

                    

                elif type_id == 'B':
                    self.box_poses.append(pose)
                    self.box_lists.append([x, y, angle_deg])
                    self.box_num += 1
                    self.known_box_num += 1

                    tf_map_box = TransformStamped()
                    tf_map_box.header.stamp = self.get_clock().now().to_msg()
                    tf_map_box.header.frame_id = 'map'
                    tf_map_box.child_frame_id = f'box_{self.box_num}'
                    tf_map_box.transform.translation.x = pose.position.x
                    tf_map_box.transform.translation.y = pose.position.y
                    tf_map_box.transform.translation.z = 0.0
                    tf_map_box.transform.rotation.x = pose.orientation.x
                    tf_map_box.transform.rotation.y = pose.orientation.y
                    tf_map_box.transform.rotation.z = pose.orientation.z
                    tf_map_box.transform.rotation.w = pose.orientation.w
                    self.static_broadcaster.sendTransform(tf_map_box)

                elif type_id == 'S':
                    starting = TransformStamped()
                    starting.header.stamp = self.get_clock().now().to_msg()
                    starting.header.frame_id = 'map'
                    starting.child_frame_id = 'odom'
                    starting.transform.translation.x = x / 100.0  # convert cm to m
                    starting.transform.translation.y = y / 100.0  # convert cm to m
                    starting.transform.translation.z = 0
                    starting.transform.rotation.x = pose.orientation.x
                    starting.transform.rotation.y = pose.orientation.y
                    starting.transform.rotation.z = pose.orientation.z
                    starting.transform.rotation.w = pose.orientation.w

                    self.static_broadcaster.sendTransform(starting)

                    self.metadata_rows.append(row)
                else:
                    self.metadata_rows.append(row)

        self.publish_arrays(self.object_poses, None, self.box_poses, None)
        # print(self.object_lists)

        # Reading workspace file to get boundary
        with open(workspace_path, mode='r', encoding='utf-8') as file:
            reader = csv.reader(file)
            # skip the first 115line (header)
            header = next(reader)
            for row in reader:
                x = int(row[0])
                y = int(row[1])
                self.boundary.append([x, y])
    
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

        self.counter = -2 # keep frames of every x frames, AND, discard first two frames

        print(41)

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

    def redetection_callback(self, msg: Point):
        # Search whole list, matching corresponding object that needs to be redetected, and set its status to False.
        for index in range(len(self.object_lists)):
            object = self.object_lists[index]
            if int(round(msg.x * 100)) == object[0] and int(round(msg.y * 100)) == object[1] and object[4]:
                object[4] = False # needs to be redetected
                object[3] = False # no actual object there
                self.get_logger().info(f"Object {index + 1} needs redetection.")
                return
    
    def success_callback(self, msg: Point):
        # Delete successfully picked up objects since it will never be used later.
        for index in range(len(self.object_lists)):
            object = self.object_lists[index]
            if int(round(msg.x * 100)) == object[0] and int(round(msg.y * 100)) == object[1] and object[4]:
                object[3] = False # no actual object there
                self.get_logger().info(f"Object {index + 1} has picked up successfully")               
                return

    def cloud_callback(self, msg: PointCloud2):
        # Spatial and color filtering, reconstructing cloud as [Timestamp, header, fields, candidates, grey_points],
        # where candidates are points (x,y,z,r,g,b) for object detection and grey_points are points (z, -x) for box detection
        
        # Publish frequency of PointCloud: 6 FPS
        num = 2  # keep one frame every 3 frames
        self.counter += 1

        # For testing the freq of cloud_callback
        # For now, it's almost 6Hz when no detection results
        # self.get_logger().info(f"self.counter: {self.counter}")

        if self.counter <= 0: # Discard first several frames, since timestamp is earlier than TF
            return
        # if self.counter % num != 0:
        #     return

        # Initialization
        candidates = []
        grey_points = []
        Timestamp = msg.header.stamp
        header = msg.header
        fields = msg.fields

        test_points_box = []
        test_points_cube = []  # for testing the point cloud range for cube detection, can be removed later

        # read original pointcloud
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        
        # color conversion into RGB (vectorized while preserving original logic)
        colors_uint32 = np.empty(points.shape[0], dtype=np.uint32)
        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            colors_uint32[idx] = ctypes.c_uint32(i).value
        
        # Extract and normalize RGB components vectorized
        r = np.asarray((colors_uint32 >> 16) & 255, dtype=np.uint8).astype(np.float32) / 255.0
        g = np.asarray((colors_uint32 >> 8) & 255, dtype=np.uint8).astype(np.float32) / 255.0
        b = np.asarray(colors_uint32 & 255, dtype=np.uint8).astype(np.float32) / 255.0
        
        # Extract coordinates
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # iterate through points and apply spatial and color filtering (vectorized implementation below)
        
        # spatial filtering for candidate points (keep points in front of camera and within 0.8m, and at the ground)
        spatial_mask = (y > 0.045) & (y < 0.0865) & (z > 0.05) & (z < 0.9)
        
        # object detection candidate points 
        object_mask = spatial_mask & (y > 0.05)
        # box detection candidate points
        box_mask_spatial = spatial_mask & (y < 0.055)
        
        # Vectorized is_grey_HSL check 
        grey_mask = self._is_grey_hsl_vectorized_from_rgb(r, g, b)
        box_mask = box_mask_spatial & grey_mask
        
        # Get indices of points that satisfy the masks
        obj_indices = np.where(object_mask)[0]
        box_indices = np.where(box_mask)[0]

        # if len(np.where(box_mask_spatial)[0]) > 0:
            # test_points_box = gen[box_mask_spatial].tolist()  # for testing the point cloud range for box detection, can be removed later

        
        # Build candidates list: (x, y, z, r, g, b) format
        if len(obj_indices) > 0:
            candidates = np.column_stack([
                x[obj_indices], y[obj_indices], z[obj_indices],
                r[obj_indices], g[obj_indices], b[obj_indices]
            ]).astype(np.float32)
            test_points_cube = gen[obj_indices].tolist()  # for testing the point cloud range for box detection, can be removed later
        else:
            candidates = np.empty((0, 6), dtype=np.float32)
            test_points_cube = []

        # Build grey_points list: (z, -x) format for box detection
        if len(box_indices) > 0:
            grey_points = np.column_stack([z[box_indices], -x[box_indices]]).astype(np.float32)
            test_points_box = gen[box_indices].tolist()
        else:
            grey_points = np.empty((0, 2), dtype=np.float32)  
            test_points_box = []
        
        # Queue the processed data
        self.cloud_queue.append([Timestamp, header, fields, candidates, grey_points, test_points_box, test_points_cube])

        # Publish test point cloud (keep original behavior)
        if test_points_box:
            box_cloud = pc2.create_cloud(header, fields, test_points_box)
            self.test_pub_box.publish(box_cloud)
        
    def process_queue(self):
        if not self.cloud_queue:
            return

        if self.cloud_queue:
            [t_cloud, header, fields, candidates, grey_points, test_points_box, test_points_cube] = self.cloud_queue[0]
            
            frame = header.frame_id

            # TODO: (Private Test) Print out the time difference between timestamp of pointcloud and latest TF
            # try:
            #     latest_tf = self.tf_buffer.lookup_transform(
            #         'odom',                  # target frame
            #         'base_link', # source frame
            #         Time()                   # latest available
            #     )
            #     latest_tf_time = latest_tf.header.stamp
            # except Exception as e:
            #     self.get_logger().warn(f"Cannot get latest TF: {e}")
            # self.get_logger().info(f"Time difference: {t_cloud.sec - latest_tf_time.sec}.{t_cloud.nanosec - latest_tf_time.nanosec}")
            # self.get_logger().info(f"Pointcloud Timestamp: {t_cloud.sec}.{t_cloud.nanosec}")
            # self.get_logger().info(f"Latest TF Timestamp: {latest_tf_time.sec}.{latest_tf_time.nanosec}")
            self.get_logger().info(f"num of queue:{len(self.cloud_queue)}")

            if self.tf_buffer.can_transform(
                'map',
                frame,
                t_cloud,
                timeout=rclpy.duration.Duration(seconds=0.1)
            ):
                self.process_point_cloud(self.cloud_queue.popleft())
            
            # Test code, finding the earliest available timestamp of TF

            # try:
            #     self.tf_buffer.lookup_transform('map', 'base_link', Time(seconds=1000, nanoseconds=1000))
            # except TransformException as e:
            #     # 错误消息格式类似：
            #     # "Lookup would require extrapolation into the past.  
            #     #  Requested time 100.000000 but the earliest data is at time 105.000000"
            #     import re
            #     match = re.search(r"earliest data is at time (\d+\.\d+)", str(e))
            #     if match:
            #         earliest_sec = float(match.group(1))
            #         earliest_time = Time(seconds=int(earliest_sec), nanoseconds=int((earliest_sec % 1) * 1e9))
            #         self.get_logger().warn(f"the earliest time is {earliest_sec}.{earliest_time.nanoseconds}")

    def process_point_cloud(self, data):

        [timestamp, header, fields, candidates, grey_points, test_points_box, test_points_cube] = data

        # TODO: (Private Test) publish the candidate points for visualization and debugging, can be removed later
        # if test_points_box:
        #     box_cloud = pc2.create_cloud(header, fields, test_points_box)
        #     self.test_pub_box.publish(box_cloud)
        if test_points_cube:
            cube_cloud = pc2.create_cloud(header, fields, test_points_cube)
            self.test_pub_cube.publish(cube_cloud)
        

        # DBSCAN for clustering object candidate points, and then color-based classification and centroid calculation for each cluster
        if candidates.shape[0] >= 8:
            pts_xyz = candidates[:, :3]

            # DBSCAN 
            eps = 0.025          # cluster radius, tuned based on the point cloud density and object size (0.025m = 2.5cm)
            min_samples = 10     # minimum number of points, ensuring each cluster contains an object
            clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(pts_xyz)
            labels = clustering.labels_

            unique_labels = set(labels) - {-1}  # ignore noise points
            for label in unique_labels:
                cluster_mask = (labels == label)
                cluster_pts = candidates[cluster_mask]   # (x,y,z,r,g,b)

                if cluster_pts.shape[0] < min_samples:
                    continue

                # Vectorized color counting
                r = cluster_pts[:, 3]
                g = cluster_pts[:, 4]
                b = cluster_pts[:, 5]
                h, s, v = self._rgb_to_hsv_vectorized(r, g, b)

                red_cnt   = np.sum(self._is_red_vectorized(h, s, v))
                blue_cnt  = np.sum(self._is_blue_vectorized(h, s, v))
                green_cnt = np.sum(self._is_green_vectorized(h, s, v))
                wood_cnt  = 0   # or use vectorized wood detection if needed

                total = cluster_pts.shape[0]
                color_counts = {'Red': red_cnt, 'Blue': blue_cnt, 'Green': green_cnt, 'Wood': wood_cnt}
                max_color = max(color_counts, key=color_counts.get)
                if color_counts[max_color] / total > 0.35:
                    # centroid (vectorized)
                    sum_x = np.sum(cluster_pts[:, 0])
                    sum_y = np.sum(cluster_pts[:, 1])
                    sum_z = np.sum(cluster_pts[:, 2])
                    counter = total
                    self.object_publish(header, timestamp, sum_x, sum_y, sum_z, counter, max_color)

        # box detection
        if isinstance(grey_points, np.ndarray) and grey_points.shape[0] > 0:
            box_size = (0.24, 0.16)  # L, W
            center, yaw, axes = self.estimate_box_from_points(grey_points, box_size)
            if center is not None:
                # publish TF in map frame
                try:
                    point_camera = PointStamped()
                    point_camera.header.frame_id = 'realsense_camera_link'
                    point_camera.header.stamp = timestamp
                    point_camera.point.x = float(center[0])
                    point_camera.point.y = float(center[1])
                    point_camera.point.z = 0.0
                    point_map = self.tf_buffer.transform(point_camera, 'map')

                    dir_camera = Vector3Stamped()
                    dir_camera.header.frame_id = 'realsense_camera_link'
                    dir_camera.header.stamp = timestamp
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

                    # whether box is within the workspace boundary
                    if not is_point_in_polygon(point_map.point.x * 100, point_map.point.y * 100, self.boundary, True):
                        self.get_logger().warn(f"box detected outside of workspace boundary, discarded, position: {point_map.point.x}, {point_map.point.y}, {angle_int}")
                        return
                    
                    # repetition check
                    for item in self.box_lists:
                        if np.abs(item[0] - x_str) < 20 and np.abs(item[1] - y_str) < 20:
                            self.get_logger().debug("repeated box detection, discarded")
                            break
                    else:
                        self.box_num += 1
                        tf_map_box = TransformStamped()
                        tf_map_box.header.stamp = timestamp
                        tf_map_box.header.frame_id = 'map'
                        tf_map_box.child_frame_id = f'box_{self.box_num}'
                        tf_map_box.transform.translation.x = point_map.point.x
                        tf_map_box.transform.translation.y = point_map.point.y
                        tf_map_box.transform.translation.z = 0
                        q = quaternion_from_euler(0.0, 0.0, angle_int * np.pi / 180)
                        tf_map_box.transform.rotation.x = q[0]
                        tf_map_box.transform.rotation.y = q[1]
                        tf_map_box.transform.rotation.z = q[2]
                        tf_map_box.transform.rotation.w = q[3]
                        self.static_broadcaster.sendTransform(tf_map_box)
                        self.get_logger().info(f'Box {self.box_num}: {x_str} {y_str} {angle_int}')

                        self.box_lists.append([x_str, y_str, angle_int])
                        new_box_msg = Pose()
                        new_box_msg.position.x = point_map.point.x
                        new_box_msg.position.y = point_map.point.y
                        new_box_msg.position.z = 0.0
                        new_box_msg.orientation.x = tf_map_box.transform.rotation.x
                        new_box_msg.orientation.y = tf_map_box.transform.rotation.y
                        new_box_msg.orientation.z = tf_map_box.transform.rotation.z
                        new_box_msg.orientation.w = tf_map_box.transform.rotation.w
                        self.publish_arrays(None, None, [new_box_msg], timestamp)

                except TransformException as ex:
                    self.get_logger().error(f'Transform failed: {ex}')

    
    def object_publish(self, header, timestamp, sum_x, sum_y, sum_z, counter, color):
        # object_num is the number of detected objects, regardless of color, used for TF frame naming
        self.get_logger().debug(f'{color} object detected.')
        self.object = tf2_geometry_msgs.PoseStamped()
        self.object.header = header
        self.object.header.stamp = timestamp
        self.object.pose.position.x = sum_x / counter
        self.object.pose.position.y = sum_y / counter
        self.object.pose.position.z = sum_z / counter
        self.object.pose.orientation.x = 0.0
        self.object.pose.orientation.y = 0.0
        self.object.pose.orientation.z = 0.0
        self.object.pose.orientation.w = 1.0

        # object_lists containes position of currently here (object[3] = True) objects
        object_lists = [(object[0], object[1]) for object in self.object_lists if object[3]]

        msg_time = timestamp
        if not self.tf_buffer.can_transform(
                'map',
                self.object.header.frame_id,
                msg_time,
                timeout=rclpy.duration.Duration(seconds=1)
            ):
                self.get_logger().warn(f'Failed to publish {color} object_{self.object_num}')
                return

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
        
        if not is_point_in_polygon(object_map.pose.position.x * 100, object_map.pose.position.y * 100, self.boundary, False):
            self.get_logger().warn(f"object detected outside of workspace boundary, discarded, position: {object_map.pose.position.x}, {object_map.pose.position.y}")
            return

        # Check if the detected object is inside any of the known boxes (with a tolerance), if yes, discard it, since objects inside boxes should not be detected
        if self.is_point_inside_any_box(object_map.pose.position.x, object_map.pose.position.y, tolerance=0.03):
            self.get_logger().debug("Object is inside a box (with tolerance), ignored.")
            return
        
        # Assign color to objects read from map file

        if self.color_unassigned_indices_list:
            for index in self.color_unassigned_indices_list:
                item = self.object_lists[index]
                if np.abs(item[0] - object_map.pose.position.x * 100) < 15 and np.abs(item[1] - object_map.pose.position.y * 100) < 15:
                    self.object_lists[index][2] = color
                    self.get_logger().info(f"object {index+1}'s color is assigned as {color}")
                    del self.color_unassigned_indices_list[index]
                    return
                
        # Extract all objects that need to be redetected, stored as indices in re_object_list.
        re_object_list = [i for i in range(len(self.object_lists)) if not self.object_lists[i][4]]
        # self.get_logger().info(f"re_object_list: {re_object_list}")
        # self.get_logger().info(f"object in map: {object_map.pose.position.x}, {object_map.pose.position.y}, color: {color}")
        
        if re_object_list:
            for index in re_object_list:
                item = self.object_lists[index]
                # Color criteria and position criteria
                if np.abs(item[0] - object_map.pose.position.x * 100) < 15 and np.abs(item[1] - object_map.pose.position.y * 100) < 15 and item[2] == color:
                    self.object_lists[index][4] = True
                    self.object_lists[index][3] = True
                    self.get_logger().info(f"Object {index + 1} redetected at position {object_map.pose.position.x}, {object_map.pose.position.y}")
                    self.object_lists[index][0] = int(round(object_map.pose.position.x * 100))
                    self.object_lists[index][1] = int(round(object_map.pose.position.y * 100))

                    new_object_msg = Pose()
                    new_object_msg.position.x = object_map.pose.position.x
                    new_object_msg.position.y = object_map.pose.position.y
                    new_object_msg.position.z = 0.0
                    new_object_msg.orientation.x = 0.0
                    new_object_msg.orientation.y = 0.0
                    new_object_msg.orientation.z = 0.0
                    new_object_msg.orientation.w = 1.0
                    self.publish_arrays([new_object_msg], msg_time, None, None)

                    tf = TransformStamped()
                    tf.header.stamp = timestamp
                    tf.header.frame_id = 'map'
                    tf.child_frame_id = f'object_{index + 1}'
                    tf.transform.translation.x = object_map.pose.position.x
                    tf.transform.translation.y = object_map.pose.position.y
                    tf.transform.translation.z = 0.0
                    tf.transform.rotation.x = 0.0
                    tf.transform.rotation.y = 0.0
                    tf.transform.rotation.z = 0.0
                    tf.transform.rotation.w = 1.0
                    self.static_broadcaster.sendTransform(tf)
                    return

        # Normal distance criteria
        for item in object_lists:
            if np.abs(item[0] - object_map.pose.position.x * 100) < 15 and np.abs(item[1] - object_map.pose.position.y * 100) < 15:
                # self.get_logger().debug(f"repeated object {self.object_lists.index(item)} detection, discarded")
                break
        else:
                       
            self.object_lists.append([int(round(object_map.pose.position.x * 100)), int(round(object_map.pose.position.y * 100)), f'{color}', True, True, int(round(object_map.pose.position.x * 100)), int(round(object_map.pose.position.y * 100))])
            new_object_msg = Pose()
            new_object_msg.position.x = object_map.pose.position.x
            new_object_msg.position.y = object_map.pose.position.y
            new_object_msg.position.z = 0.0
            new_object_msg.orientation.x = 0.0
            new_object_msg.orientation.y = 0.0
            new_object_msg.orientation.z = 0.0
            new_object_msg.orientation.w = 1.0
            self.publish_arrays([new_object_msg], msg_time, None, None)
            self.object_num += 1

            tf = TransformStamped()
            tf.header.stamp = timestamp
            tf.header.frame_id = 'map'
            tf.child_frame_id = f'object_{self.object_num}'
            tf.transform.translation.x = object_map.pose.position.x
            tf.transform.translation.y = object_map.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
            tf.transform.rotation.w = 1.0
            self.static_broadcaster.sendTransform(tf)

            self.get_logger().info(f'Object {self.object_num}: {color} {object_map.pose.position.x} {object_map.pose.position.y} N/A')
            # print(f"z distance: {sum_z / counter:.3f} m")

    def is_point_inside_any_box(self, x, y, tolerance=0.03):
        """
        Check if an object at (x, y) is inside any of the detected and known boxes, considering the box dimensions and a tolerance.
        L, W: box dimensions (meters), for example L=0.24, W=0.16
        tolerance: expansion distance (meters), for example 0.03
        """
        L = 0.24
        W = 0.16
        half_L = L / 2.0 + tolerance
        half_W = W / 2.0 + tolerance
        for box in self.box_lists:
            cx = box[0] / 100.0
            cy = box[1] / 100.0
            angle_deg = box[2]
            angle_rad = math.radians(angle_deg)
            dx = x - cx
            dy = y - cy
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            local_x = dx * cos_a + dy * sin_a
            local_y = -dx * sin_a + dy * cos_a
            if abs(local_x) <= half_L and abs(local_y) <= half_W:
                return True
        return False
        
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
        
        if len(points) < 150: 
            return None, None, None 
        
        # self.get_logger().info(f"the length of points: {len(points)}")
        pts = np.asarray(points, dtype=np.float32)

        # --- Step 1: PCA --- 
        mean = np.mean(pts, axis=0) 
        pts_centered = pts - mean 
        _, S, Vt = np.linalg.svd(pts_centered, full_matrices=False) 
        axes = Vt[:2] 
        
        # Step 3: two edge vs single edge decision based on variance ratio
        ratio = S[1] / S[0] 

        # self.get_logger().info(f"ratio: {ratio}")

        if ratio > 0.1:
            # =========================================================
            # RANSAC - two edges (Vectorized version)
            # =========================================================

            pts_np = pts.copy()

            # -------------------------
            # first edge
            # -------------------------
            # Visulize Initialization
            lines_viz = []
            timestamp_viz = self.get_clock().now().to_msg()

            model1, mask1 = self._ransac_lines_vectorized(pts_np)
            if model1 is None or mask1.sum() < 10:
                # self.get_logger().warn("Line1 RANSAC failed: insufficient inliers")
                # lines_viz.append({'status':'failed'})
                # # publish pointcloud
                # self.publish_ransac_lines(pts_np, lines_viz, timestamp_viz)
                return None, None, None
            else:
                # RANSAC successful
                n1, d1 = model1
                dir1 = np.array([n1[1], -n1[0]])
                norm1 = np.linalg.norm(dir1)
                if norm1 > 1e-6: dir1 /= norm1
                centroid = np.mean(pts_np, axis=0)
                denom = np.dot(n1, dir1)
                if abs(denom) > 1e-6:
                    t = -(np.dot(n1, centroid) + d1) / denom
                    pt_on_line = centroid + t * dir1
                else:
                    pt_on_line = np.array([0.0, 0.0])  # fallback
                # lines_viz.append({'status':'success', 'dir':dir1, 'point':pt_on_line, 'color':(1.0,0.0,0.0)})

            # -------------------------
            # second edge
            # -------------------------
            remaining = pts_np[~mask1]
            model2, mask2 = self._ransac_lines_vectorized(remaining)
            if model2 is None or mask2.sum() < 8:
                # self.get_logger().warn("Line2 RANSAC failed: insufficient inliers")
                # lines_viz.append({'status':'failed'})
                # # Publish First Line
                # self.publish_ransac_lines(pts_np, lines_viz, timestamp_viz)
                return None, None, None
            else:
                n2, d2 = model2
                dir2 = np.array([n2[1], -n2[0]])
                norm2 = np.linalg.norm(dir2)
                if norm2 > 1e-6: dir2 /= norm2
                centroid2 = np.mean(remaining, axis=0)
                denom2 = np.dot(n2, dir2)
                if abs(denom2) > 1e-6:
                    t2 = -(np.dot(n2, centroid2) + d2) / denom2
                    pt_on_line2 = centroid2 + t2 * dir2
                else:
                    pt_on_line2 = np.array([0.0, 0.0])
                # lines_viz.append({'status':'success', 'dir':dir2, 'point':pt_on_line2, 'color':(0.0,1.0,1.0)})

            # Visulization on RANSAC
            # self.publish_ransac_lines(pts_np, lines_viz, timestamp_viz)

            # corner point
            n1, d1 = model1
            n2, d2 = model2

            A = np.vstack([n1, n2])
            b = -np.array([d1, d2])
            corner = np.linalg.pinv(A) @ b

            # =========================================================
            # direction vectors and used axes 
            # =========================================================
            dir1 = np.array([n1[1], -n1[0]])
            dir2 = np.array([n2[1], -n2[0]])

            norm1 = np.linalg.norm(dir1)
            norm2 = np.linalg.norm(dir2)

            if norm1 < 1e-6 or norm2 < 1e-6:
                return None, None, None

            dir1 /= norm1
            dir2 /= norm2

            pts_corner = pts_np - corner
            proj1 = pts_corner @ dir1
            proj2 = pts_corner @ dir2
            if np.median(proj1) < 0:
                dir1 = -dir1
            if np.median(proj2) < 0:
                dir2 = -dir2

            used_axes = np.vstack([dir1, dir2])

            # =========================================================
            # Emunerate all possible center points and validates (vectorized)
            # =========================================================
            L, W = box_size

            dirs = np.stack([[dir1, dir2], [dir2, dir1]])  # (2,2,2)
            signs = np.array([[1,1],[1,-1],[-1,1],[-1,-1]])

            centers = (
                corner
                + dirs[:,0][:,None,:] * signs[None,:,0:1] * (L/2)
                + dirs[:,1][:,None,:] * signs[None,:,1:2] * (W/2)
            ).reshape(-1,2)

            yaws = np.arctan2(dirs[:,0][:,None,1], dirs[:,0][:,None,0]).repeat(4, axis=1).reshape(-1)

            scores = self.compute_band_score_batch(pts_np, centers, yaws, L, W)

            best_idx = np.argmax(scores)

            self.get_logger().debug(f"score: {scores[best_idx]}")

            if scores[best_idx] < 0.8:
                return None, None, None

            center_shifted = centers[best_idx]
            yaw = yaws[best_idx]

            self.get_logger().debug(
                f'Corner: {corner}, Center: {center_shifted}, yaw: {yaw:.3f}, score: {scores[best_idx]:.3f}'
            )

            return center_shifted, yaw, used_axes

        else:
            # =========================================================
            # single edge case - use PCA axes, shift center along normal
            # =========================================================

            # define reference x-axis
            x_axis = np.array([1.0, 0.0])

            # principal directions
            dir1 = axes[0]
            dir2 = axes[1]

            # normalize (safety)
            dir1 = dir1 / (np.linalg.norm(dir1) + 1e-8)
            dir2 = dir2 / (np.linalg.norm(dir2) + 1e-8)

            # compute angles to x-axis
            angle_dir1_x = np.arctan2(dir1[1], dir1[0])
            angle_dir2_x = np.arctan2(dir2[1], dir2[0])

            self.get_logger().debug(
                f'dir1 与 x 轴夹角: {angle_dir1_x:.2f} rad, dir2 与 x 轴夹角: {angle_dir2_x:.2f} rad'
            )

            # choose normal direction (pointing roughly +x)
            normal = dir2 if np.dot(dir2, x_axis) > 0 else -dir2

            # project points onto main axis (only dir1 used)
            projected = pts_centered @ dir1

            # min_proj = projected.min()
            # max_proj = projected.max()

            # center_proj = (min_proj + max_proj) / 2.0
            # center = mean + center_proj * dir1

            # length_proj = max_proj - min_proj

            q_low  = np.percentile(projected, 5)
            q_high = np.percentile(projected, 95)

            center_proj = (q_high + q_low) / 2.0
            center = mean + center_proj * dir1

            length_proj = float(q_high - q_low)
            width_proj = length_proj  # placeholder (same as old logic)

            # self.get_logger().info(
            #     f'length_proj: {length_proj:.3f}'
            # )

            # Exclude noise background
            if length_proj <= 0.1:
                return None, None, None

            # assign box dimensions
            if length_proj >= width_proj:
                box_length = box_size[0]
                box_width = box_size[1]
            else:
                # swap (rare here, but keep consistency)
                dir1, dir2 = dir2, dir1
                normal = dir2 if np.dot(dir2, x_axis) > 0 else -dir2
                box_length = box_size[1]
                box_width = box_size[0]

            # decide shifting magnitude + yaw
            if length_proj >= box_width:
                shift_vec = normal * (box_width / 2.0)
                yaw = angle_dir1_x
            else:
                shift_vec = normal * (box_length / 2.0)
                yaw = angle_dir1_x - np.pi / 2

            center_shifted = center + shift_vec

            used_axes = np.vstack([dir1, normal])

            self.get_logger().debug(
                f'[Single Edge] center: {center_shifted}, yaw: {yaw:.3f}'
            )

            return center_shifted, yaw, used_axes
        
    # === RANSAC VISUALIZATION (Simplified) ===
    def publish_ransac_lines(self, pts, lines, timestamp):
        """
        Publish only the fitted RANSAC lines.
        pts: Nx2 array of all points (for background)
        lines: list of dict, each with:
            - 'dir': direction vector (2,)
            - 'point': a point on the line (2,)
            - 'color': (r,g,b) tuple
            - 'status': 'success' or 'failed'
        """
        marker_array = MarkerArray()
        
        # Helper
        def to_point(p):
            return Point(x=float(p[0]), y=float(p[1]), z=0.0)
        
        # Background points (grey)
        bg_marker = Marker()
        bg_marker.header.frame_id = "realsense_camera_link"
        bg_marker.header.stamp = timestamp
        bg_marker.ns = "ransac_points"
        bg_marker.id = 0
        bg_marker.type = Marker.POINTS
        bg_marker.action = Marker.ADD
        bg_marker.scale.x = 0.005
        bg_marker.scale.y = 0.005
        bg_marker.color.a = 0.5
        bg_marker.color.r = 0.7
        bg_marker.color.g = 0.7
        bg_marker.color.b = 0.7
        for p in pts:
            bg_marker.points.append(to_point(p))
        marker_array.markers.append(bg_marker)
        
        # Draw each successful line
        line_id = 1
        for line in lines:
            if line['status'] != 'success':
                continue
            dir_vec = line['dir']
            point_on_line = line['point']
            # Extend line in both directions
            line_len = 0.5  # meters
            p_start = point_on_line - dir_vec * line_len
            p_end   = point_on_line + dir_vec * line_len
            
            line_marker = Marker()
            line_marker.header = bg_marker.header
            line_marker.ns = "ransac_lines"
            line_marker.id = line_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.scale.x = 0.01
            line_marker.color.a = 1.0
            line_marker.color.r, line_marker.color.g, line_marker.color.b = line['color']
            line_marker.points = [to_point(p_start), to_point(p_end)]
            marker_array.markers.append(line_marker)
            line_id += 1
        
        self.ransac_viz_pub.publish(marker_array)
        
    def _ransac_lines_vectorized(self, points, n_samples=256, threshold=0.012):

        if len(points) < 2:
            return None, None

        N = points.shape[0]

        idx = np.random.randint(0, N, (n_samples, 2))
        p1 = points[idx[:, 0]]
        p2 = points[idx[:, 1]]

        dirs = p2 - p1
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        valid = norms[:, 0] > 1e-6

        if np.sum(valid) < 5:
            return None, None 

        dirs[valid] /= norms[valid]

        normals = np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)
        d = -np.sum(normals * p1, axis=1)

        dist = np.abs(points @ normals.T + d)

        inliers = dist < threshold
        scores = inliers.sum(axis=0)

        best = np.argmax(scores)

        return (normals[best], d[best]), inliers[:, best]
        
    def compute_band_score_batch(self, pts, centers, yaws, L, W, band_width=0.015):

        if pts.shape[0] < 50:
            return np.zeros(len(centers))

        c = np.cos(yaws)
        s = np.sin(yaws)

        # rotation matrices (K,2,2)
        R = np.stack([
            np.stack([c, s], axis=1),
            np.stack([-s, c], axis=1)
        ], axis=1)

        # transform to local frame
        pts_shifted = pts[None, :, :] - centers[:, None, :]
        pts_local = np.einsum('kij,knj->kni', R, pts_shifted)

        x = pts_local[:, :, 0]
        y = pts_local[:, :, 1]

        half_L = L / 2.0
        half_W = W / 2.0
        d = band_width

        # four edge bands
        right  = (x >= half_L - d) & (x <= half_L + d) & (y >= -half_W - d) & (y <= half_W + d)
        left   = (x >= -half_L - d) & (x <= -half_L + d) & (y >= -half_W - d) & (y <= half_W + d)
        top    = (y >= half_W - d) & (y <= half_W + d) & (x >= -half_L - d) & (x <= half_L + d)
        bottom = (y >= -half_W - d) & (y <= -half_W + d) & (x >= -half_L - d) & (x <= half_L + d)

        in_band = right | left | top | bottom

        return np.nan_to_num(np.mean(in_band, axis=1))
        
    def _is_grey_hsl_vectorized_from_rgb(self, r, g, b):
        """
        Vectorized implementation of the original is_grey_HSL(r,g,b) function.
        Original logic:
            c_max = max(r,g,b); c_min = min(r,g,b); delta = c_max - c_min
            Compute Hue (h) — only condition h > 80 or h == 0 is used.
            Compute Lightness l = (c_max + c_min)/2
            Compute HSL Saturation: s = 0 if delta == 0 else delta/(1-abs(2*l-1))
            Returns: (h > 80 or h == 0) and s < 20/255 and l < 25/255
        """
        c_max = np.maximum(np.maximum(r, g), b)
        c_min = np.minimum(np.minimum(r, g), b)
        delta = c_max - c_min
        
        # Compute Hue (0-360)
        h = np.zeros_like(r)
        # Only compute where delta != 0 to avoid division by zero
        mask_r = (delta != 0) & (c_max == r)
        h[mask_r] = 60.0 * (((g[mask_r] - b[mask_r]) / delta[mask_r]) % 6)
        mask_g = (delta != 0) & (c_max == g)
        h[mask_g] = 60.0 * (((b[mask_g] - r[mask_g]) / delta[mask_g]) + 2)
        mask_b = (delta != 0) & (c_max == b)
        h[mask_b] = 60.0 * (((r[mask_b] - g[mask_b]) / delta[mask_b]) + 4)
        
        # Compute HSL Lightness
        l = (c_max + c_min) / 2.0
        
        # Compute HSL Saturation
        s_hsl = np.zeros_like(r)
        mask_delta = delta != 0
        # s = delta / (1 - abs(2*l - 1))
        denominator = 1.0 - np.abs(2.0 * l[mask_delta] - 1.0)
        # Avoid division by zero
        denominator = np.where(denominator == 0, 1e-10, denominator)
        s_hsl[mask_delta] = delta[mask_delta] / denominator
        
        # Final grey condition
        grey_cond = ((h > 170) & (h < 270) | (h == 0)) & (s_hsl < 0.2) & (l < 0.3)
        return grey_cond
    
    def _rgb_to_hsv_vectorized(self, r, g, b):
        """Vectorized RGB to HSV conversion."""
        c_max = np.maximum(np.maximum(r, g), b)
        c_min = np.minimum(np.minimum(r, g), b)
        delta = c_max - c_min

        h = np.zeros_like(r)
        # Red is max
        mask_r = (delta != 0) & (c_max == r)
        h[mask_r] = 60.0 * (((g[mask_r] - b[mask_r]) / delta[mask_r]) % 6)
        # Green is max
        mask_g = (delta != 0) & (c_max == g)
        h[mask_g] = 60.0 * (((b[mask_g] - r[mask_g]) / delta[mask_g]) + 2)
        # Blue is max
        mask_b = (delta != 0) & (c_max == b)
        h[mask_b] = 60.0 * (((r[mask_b] - g[mask_b]) / delta[mask_b]) + 4)

        s = np.zeros_like(r)
        mask_cmax = c_max != 0
        s[mask_cmax] = delta[mask_cmax] / c_max[mask_cmax]
        v = c_max
        return h, s, v

    def _is_red_vectorized(self, h, s, v):
        return ((h <= 25) | (h >= 335)) & (s > 0.4) & (v > 0.2)

    def _is_blue_vectorized(self, h, s, v):
        return (h >= 185) & (h <= 220) & (s > 0.4) & (v > 0.2)

    def _is_green_vectorized(self, h, s, v):
        return (h >= 140) & (h <= 185) & (s > 0.4) & (v > 0.2)
    
    def write_csv(self):
        try:
            with open(self.output_map_path, mode='w', encoding='utf-8', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Type', 'x', 'y', 'angle'])
                for meta_row in self.metadata_rows:
                    writer.writerow(meta_row)
                for obj in self.object_lists:
                    writer.writerow(['O'] + obj[5:7])
                for box in self.box_lists:
                    writer.writerow(['B'] + box)
            self.get_logger().debug(f'CSV file updated: {self.output_map_path}')
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

#######################################################################
# TODO: Should not be modified, Andrew referred rgb_to_hsv() function.
#######################################################################

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

# Checking if a point is a valid one, which means it is within the boundary of the workspace
def is_point_in_polygon(x, y, polygon, if_box = False):
    if not if_box:
        n = len(polygon)
        inside = False

        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[(i + 1) % n]

            intersect = ((yi > y) != (yj > y)) and \
                        (x < xi + (y - yi) * (xj - xi) / (yj - yi))

            if intersect:
                inside = not inside

        return inside
    else:
        new_polygon = shrink_polygon(polygon, 6) # Therotically 8 cm, considering position error of boxes
        n = len(new_polygon)
        inside = False

        for i in range(n):
            xi, yi = new_polygon[i]
            xj, yj = new_polygon[(i + 1) % n]

            intersect = ((yi > y) != (yj > y)) and \
                        (x < xi + (y - yi) * (xj - xi) / (yj - yi))

            if intersect:
                inside = not inside

        return inside

def shrink_polygon(polygon, d):
    """
    polygon: Nx2 array (counter-clockwise)
    d: shrink distance in cm
    """
    polygon = np.array(polygon, dtype=float)
    n = len(polygon)
    new_polygon = []

    for i in range(n):
        p_prev = polygon[i - 1]
        p_curr = polygon[i]
        p_next = polygon[(i + 1) % n]

        # edge directions
        e1 = p_curr - p_prev
        e2 = p_next - p_curr

        # normals (pointing inward)
        n1 = np.array([-e1[1], e1[0]])
        n2 = np.array([-e2[1], e2[0]])

        n1 = n1 / np.linalg.norm(n1)
        n2 = n2 / np.linalg.norm(n2)

        # shift lines
        p1_shift = p_prev + n1 * d
        p2_shift = p_curr + n2 * d

        # line intersection
        A = np.vstack([n1, n2])
        b = np.array([np.dot(n1, p1_shift), np.dot(n2, p2_shift)])

        try:
            x = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            x = np.linalg.pinv(A) @ b

        new_polygon.append(x)

    return np.array(new_polygon)

if __name__ == '__main__':
    main()
