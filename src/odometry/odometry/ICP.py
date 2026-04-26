#!/usr/bin/env python3

import csv

import rclpy
from rclpy.node import Node
from pathlib import Path

import numpy as np

import rclpy.time
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

import tf_transformations
import tf2_ros

from sklearn.neighbors import NearestNeighbors

base_dir = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent.parent
KNOWN_PATH = base_dir / 'Workspace/map_1_1.csv'

def scan_to_points(scan):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    points = []

    for r, a in zip(scan.ranges, angles):
        if np.isfinite(r):
            x = r * np.cos(a)
            y = r * np.sin(a)
            points.append([x, y])

    return np.array(points)

def icp(source, target, max_iter=20):
    T = np.eye(3)

    for _ in range(max_iter):
        nbrs = NearestNeighbors(n_neighbors=1).fit(target)
        distances, indices = nbrs.kneighbors(source)
        matched = target[indices[:, 0]]

        mu_s = np.mean(source, axis=0)
        mu_t = np.mean(matched, axis=0)

        S = source - mu_s
        M = matched - mu_t

        W = M.T @ S
        U, _, VT = np.linalg.svd(W)
        R = U @ VT

        t = mu_t - R @ mu_s

        source = (R @ source.T).T + t

        T_step = np.eye(3)
        T_step[:2, :2] = R
        T_step[:2, 2] = t

        T = T_step @ T

    return T

class ICPMapper(Node):

    def __init__(self):
        super().__init__('icp_mapper')

        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.scan_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_sub = self.create_subscription(
            String, '/command', self.cmd_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish_tf)
        
        with open(KNOWN_PATH, newline='', encoding='utf-8-sig') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                if row['Type'] == 'S':
                    type = str(row['Type'])
                    x = float(row['x'])/100  # convert to meters
                    y = float(row['y'])/100
                    angle = float(row['angle'])
                    start = (type, x, y, angle)

        self.last_scan = None
        self.last_stamp = None
        self.map_points = None
        self.last_odom_pose = None

        start_x = start[1] # type: ignore
        start_y = start[2] #type: ignore
        start_yaw = start[3] #type: ignore

        self.T_map_odom = np.eye(3)

        self.T_map_odom[0, 2] = start_x
        self.T_map_odom[1, 2] = start_y

        self.T_map_odom[0, 0] = np.cos(start_yaw)
        self.T_map_odom[0, 1] = -np.sin(start_yaw)
        self.T_map_odom[1, 0] = np.sin(start_yaw)
        self.T_map_odom[1, 1] = np.cos(start_yaw)
        
        self.current_odom_pose = None
        self.publish_tf()
        
    def scan_cb(self, msg):
        self.last_scan = scan_to_points(msg)
        self.last_stamp = msg.header.stamp
        
    def get_odom_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',        # target frame
                'base_link',   # source frame
                rclpy.time.Time(seconds=0)
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            self.get_logger().info(f"Odom pose:\n{x:.2f}, {y:.2f}")

            q = trans.transform.rotation
            yaw = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])[2]

            return np.array([x, y, yaw])

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        
    def cmd_cb(self, msg):
        if msg.data == "start":
            self.start_mapping()

        elif msg.data == "correct":
            self.correct_pose()
            
    def start_mapping(self):
        self.current_odom_pose = self.get_odom_pose()
        
        if self.last_scan is None or self.current_odom_pose is None:
            self.get_logger().warn("Missing scan or odom")
            return

        self.map_points = self.last_scan.copy()
        self.last_odom_pose = self.current_odom_pose.copy()

        self.get_logger().info("Map initialized")
        
    def correct_pose(self):
        self.current_odom_pose = self.get_odom_pose()
        
        if self.map_points is None or self.last_scan is None or self.current_odom_pose is None:
            self.get_logger().warn("Missing data")
            return

        if self.last_odom_pose is None:
            self.get_logger().warn("No previous odom reference")
            return

        # --- 1. Compute odometry delta ---
        dx = self.current_odom_pose[0]# - self.last_odom_pose[0]
        dy = self.current_odom_pose[1]# - self.last_odom_pose[1]
        dtheta = self.current_odom_pose[2]# - self.last_odom_pose[2]

        R_odom = np.array([
            [np.cos(dtheta), -np.sin(dtheta)],
            [np.sin(dtheta),  np.cos(dtheta)]
        ])
        t_odom = np.array([dx, dy])

        # --- 2. Apply odometry as initial guess ---
        source = self.last_scan.copy()
        source = (R_odom @ source.T).T + t_odom

        target = self.map_points.copy()

        # --- 3. Run ICP ---
        T_icp = icp(source, target)

        self.get_logger().info(f"ICP correction:\n{T_icp}")

        # --- 4. Combine transforms ---
        T_odom = np.eye(3)
        T_odom[:2, :2] = R_odom
        T_odom[:2, 2] = t_odom

        T_total = T_icp @ T_odom

        self.T_map_odom = T_total @ self.T_map_odom

        # --- 5. Transform scan into map frame ---
        R_total = T_total[:2, :2]
        t_total = T_total[:2, 2]

        transformed_scan = (R_total @ self.last_scan.T).T + t_total

        # --- 6. Downsample BEFORE adding ---
        #transformed_scan = self.downsample(transformed_scan)

        # --- 7. Add to map ---
        self.map_points = np.vstack((self.map_points, transformed_scan))

        # --- 8. Downsample whole map (important!) ---
        #self.map_points = self.downsample(self.map_points)

        self.get_logger().info(f"Map size: {self.map_points.shape[0]}")

        # --- 9. Update odom reference ---
        self.last_odom_pose = self.current_odom_pose.copy()

    #def downsample(self, points, voxel_size=0.1):
    #    if len(points) == 0:
    #        return points
#
    #    # Quantize points into grid
    #    grid = np.floor(points / voxel_size)
#
    #    # Keep one point per grid cell
    #    _, unique_indices = np.unique(grid, axis=0, return_index=True)
#
    #    return points[unique_indices]

    def publish_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg() #self.last_stamp or self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        x = self.T_map_odom[0, 2]
        y = self.T_map_odom[1, 2]

        theta = np.arctan2(
            self.T_map_odom[1, 0],
            self.T_map_odom[0, 0]
        )

        quat = tf_transformations.quaternion_from_euler(0, 0, theta)

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ICPMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()