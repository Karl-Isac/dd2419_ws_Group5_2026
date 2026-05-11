#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from pathlib import Path

import numpy as np
import rclpy.time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import open3d as o3d

from ament_index_python.packages import get_package_share_directory
import os

package_path = get_package_share_directory('detection')
KNOWN_PATH = os.path.join(package_path, 'config', 'map.csv')

# ---------------- ICP ----------------

def scan_to_points(scan):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    points = []

    for r, a in zip(scan.ranges, angles):
        if np.isfinite(r):
            points.append([r * np.cos(a), r * np.sin(a)])

    return np.array(points)

# ---------------- NODE ----------------

class ICPMapper(Node):

    def __init__(self):
        super().__init__('icp_mapper')

        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.scan_cb, 10)

        self.cmd_sub = self.create_subscription(
            String, "/localization/start_update_ICP", self.cmd_cb, 10)

        
        self.imu_sub = self.create_subscription(
            String, '/phidgets/imu/data_raw', self.imu_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- load start pose --------
        with open(KNOWN_PATH, newline='', encoding='utf-8-sig') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                if row['Type'] == 'S':
                    start = (
                        row['Type'],
                        float(row['x']) / 100,
                        float(row['y']) / 100,
                        float(row['angle'])
                    )

        start_x, start_y, start_yaw = start[1], start[2], start[3] # type: ignore

        self.T_map_odom = np.eye(3)
        self.T_map_odom[:2, :2] = [
            [np.cos(start_yaw*np.pi/180), -np.sin(start_yaw*np.pi/180)],
            [np.sin(start_yaw*np.pi/180),  np.cos(start_yaw*np.pi/180)]
        ]
        self.T_map_odom[0, 2] = start_x
        self.T_map_odom[1, 2] = start_y

        # -------- state --------
        self.raw_scan = None
        self.last_scan = None
        self.pre_localization_scan = None
        self.map_points = None
        self.pc_fix_map = o3d.geometry.PointCloud()
        self.pre_map_points = None
        self.last_stamp = None
        self.last_odom = self.T_map_odom
        self.last_angular_velocity = 0
        
        self.make_images = False
        self.temp_incrementer = 0
        self.temp_incrementer2 = 0

        self.publish_tf()

    # ---------------- callbacks ----------------
    
    def scan_cb(self, msg):
        if abs(self.last_angular_velocity) > 0.1:
            return
        
        self.raw_scan = scan_to_points(msg) # lidar_link frame
        self.last_stamp = msg.header.stamp

    def fix_scan(self):
        if self.raw_scan is None or (pose := self.get_odom_to_base()) is None:
            return

        x, y, yaw = pose

        # --- build rotation ---
        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])

        t = np.array([x, y])

        # --- transform scan into odom frame ---
        scan_odom = (R @ self.raw_scan.T).T + t

        self.last_scan = scan_odom

    def imu_cb(self, msg):
        self.last_angular_velocity = msg.angular_velocity.z

    def cmd_cb(self, msg):
        if msg.data == "start":
            self.start_mapping()
        elif msg.data == "correct":
            self.correct_pose()
        elif msg.data == "pre":
            self.fix_scan()
            self.pre_localization_scan = self.last_scan
        elif msg.data == "export":
            self.export_map_to_csv()
        else:
            self.get_logger().info("Not a valid string message")
            
    def get_odom_to_base(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",          # target frame
                "lidar_link",     # source frame
                rclpy.time.Time()  # latest available
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            return x, y, yaw

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    # ---------------- mapping ----------------

    def start_mapping(self):
        #self.fix_scan() # Put in map-frame instead
        if self.raw_scan is None:
            return
        
        try:
            t = self.tf_buffer.lookup_transform(
                "map",          # target frame
                "lidar_link",     # source frame
                rclpy.time.Time()  # latest available
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])

        t = np.array([x, y])

        # --- transform scan into map frame ---
        scan_map = (R @ self.raw_scan.T).T + t
        
        self.map_points = np.vstack((self.map_points, scan_map)) if self.map_points is not None else scan_map
        self.pc_fix_map.points = o3d.utility.Vector3dVector(to_3d(self.map_points))
        
        self.get_logger().info("Map initialized or updated")

    # ---------------- ICP correction ----------------

    def correct_pose(self):
        self.fix_scan()
        if self.map_points is None or self.last_scan is None:
            return

        if self.pre_localization_scan is not None:
            self.get_logger().info("Using pre-localization scan for ICP")
            pc_fix = o3d.geometry.PointCloud()
            pc_fix.points = o3d.utility.Vector3dVector(to_3d(self.pre_localization_scan))
            self.pre_localization_scan = None
        else:
            pc_fix = self.pc_fix_map
            
        # ---------------- convert to Open3D point clouds ----------------

        pc_mov = o3d.geometry.PointCloud()
        pc_mov.points = o3d.utility.Vector3dVector(to_3d(self.last_scan))

        # ---------------- run ICP ----------------
        icp_success = True
        try:
            reg = o3d.pipelines.registration.registration_icp(
                pc_mov,
                pc_fix,
                max_correspondence_distance=0.6, # Sorry for magic number, but this seem good
                init=np.eye(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            H = reg.transformation  # 4x4 SE(3)

        except Exception as e:
            self.get_logger().warn(f"ICP failed: {e}")
            return
        
        if reg.fitness < 0.9:
            self.get_logger().warn(f"ICP rejected (low fitness: {reg.fitness:.2f})")
            
            if self.make_images:
                self.temp_incrementer2 += 1
                if self.temp_incrementer2%5 == 0:
                    self.export_map_to_csv(filename=f"bad_scan_points_{self.temp_incrementer2/5}.csv", points=self.last_scan)
            #Above is for debugging, to see what kind of scans are rejected by ICP, and if there is a pattern to it (e.g. too few points, or points in a certain area)     
            return
        # Also for debugging vvvvv
        elif self.make_images:
            self.temp_incrementer += 1
            self.export_map_to_csv(filename=f"scan_points_{self.temp_incrementer}.csv", points=self.last_scan) # export the map used for ICP, for visualization and debugging

        # ---------------- convert SE3 → SE2 ----------------
        T_icp_se2 = se3_to_se2(H)

        # ---------------- extract motion ----------------
        delta_T = np.linalg.inv(self.T_map_odom) @ T_icp_se2
        dx = delta_T[0, 2]
        dy = delta_T[1, 2]

        trans_err = np.sqrt(dx**2 + dy**2)

        dtheta = np.arctan2(
            delta_T[1, 0],
            delta_T[0, 0]
        )

        # ---------------- apply update ----------------
        # reject large jumps, adjust thresholds as needed
        if trans_err > 0.5:
            self.get_logger().warn(f"ICP rejected (too large motion): {trans_err:.2f}m")
            icp_success = False
            
        if abs(dtheta) > np.radians(20):
            self.get_logger().warn(f"ICP rejected (too large rotation: {dtheta:.2f} rad)")
            icp_success = False

        if icp_success:
            self.T_map_odom = T_icp_se2
            self.get_logger().info(
                f"ICP accepted: Δx={dx:.2f}, Δy={dy:.2f}, err={trans_err:.2f}, Δθ={dtheta:.2f} rad, fitness={reg.fitness:.2f}")
        
    # ---------------- TF ----------------

    def publish_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        x = self.T_map_odom[0, 2]
        y = self.T_map_odom[1, 2]

        theta = np.arctan2(
            self.T_map_odom[1, 0],
            self.T_map_odom[0, 0]
        )

        q = tf_transformations.quaternion_from_euler(0, 0, theta)

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        
    def export_map_to_csv(self, filename="map_points.csv", points=None):
        if points is None:
            points = self.map_points

        if points is None or len(points) == 0:
            self.get_logger().warn("No map points to export")
            return

        try:
            np.savetxt(filename, points, delimiter=",", header="x,y", comments="")
            self.get_logger().info(f"Map exported to {filename} ({len(points)} points)")
        except Exception as e:
            self.get_logger().error(f"Failed to export map: {e}")


# ---------------- main ----------------
def to_3d(points):
    return np.hstack((points, np.zeros((points.shape[0], 1))))

def se3_to_se2(H):
    """
    Convert SE(3) homogeneous transform (4x4) → SE(2) transform (3x3)
    by extracting yaw + x,y translation.
    """

    # --- extract yaw from SE3 rotation matrix ---
    yaw = np.arctan2(H[1, 0], H[0, 0])

    # --- build SE2 rotation ---
    T = np.eye(3)
    T[:2, :2] = [
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ]

    # --- extract translation (ignore z) ---
    T[0, 2] = H[0, 3]
    T[1, 2] = H[1, 3]

    return T

def se2_to_se3(T):
    """
    Convert SE(2) (3x3) → SE(3) (4x4)
    """

    H = np.eye(4)

    # rotation (embed in XY plane)
    H[0:2, 0:2] = T[0:2, 0:2]

    # translation
    H[0, 3] = T[0, 2]
    H[1, 3] = T[1, 2]

    return H

def main(args=None):
    rclpy.init(args=args)
    node = ICPMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
