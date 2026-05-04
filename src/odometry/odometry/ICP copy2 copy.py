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


base_dir = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent.parent
KNOWN_PATH = base_dir / 'Workspace/map_1_1.csv'

'''
This ICP adds lidar scans when told to, and correct those that find a good match
'''


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
            String, '/command', self.cmd_cb, 10)
        
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
            [np.cos(start_yaw), -np.sin(start_yaw)],
            [np.sin(start_yaw),  np.cos(start_yaw)]
        ]
        self.T_map_odom[0, 2] = start_x
        self.T_map_odom[1, 2] = start_y

        # -------- state --------
        self.last_scan = None
        self.map_points = None
        self.last_stamp = None
        self.last_angular_velocity = 0
        self.last_odom_pose = self.T_map_odom.copy()

        self.publish_tf()

    # ---------------- callbacks ----------------

    def scan_cb(self, msg):
        self.last_scan = scan_to_points(msg)
        self.last_stamp = msg.header.stamp
        if self.last_angular_velocity < 0.1: # only update when robot is not rotating fast, otherwise ICP will fail
            self.map_points = np.vstack((self.map_points, self.last_scan)) if self.map_points is not None else self.last_scan
        if self.map_points is not None and len(self.map_points) > 20000:
            self.voxel_downsample()

    def imu_cb(self, msg):
        self.last_angular_velocity = msg.angular_velocity.z

    def cmd_cb(self, msg):
        if msg.data == "start":
            self.start_mapping()
        elif msg.data == "correct":
            self.correct_pose()
            
    def get_odom_to_base(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",          # target frame
                "base_link",     # source frame
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
        if self.last_scan is None:
            self.get_logger().warn("No scan yet")
            return

        self.map_points = self.last_scan.copy()
        self.get_logger().info("Map initialized")

    # ---------------- ICP correction ----------------

    def correct_pose(self):
        if self.map_points is None or self.last_scan is None:
            return

        # ---------------- convert to Open3D point clouds ----------------
        pc_fix = o3d.geometry.PointCloud()
        pc_fix.points = o3d.utility.Vector3dVector(to_3d(self.map_points))

        pc_mov = o3d.geometry.PointCloud()
        pc_mov.points = o3d.utility.Vector3dVector(to_3d(self.last_scan))

        self.get_logger().info(
            f"Map size: {len(self.map_points)}, Scan size: {len(self.last_scan)}"
        )
        
        x, y, _ = self.get_odom_to_base() or (0, 0, 0)
        delta_len = np.sqrt(x**2 + y**2)
        
        max_angle_error = np.radians(15)
        
        delta_alpha = delta_len*np.tan(max_angle_error)

        # ---------------- run ICP ----------------
        icp_success = True
        try:
            reg = o3d.pipelines.registration.registration_icp(
                pc_mov,
                pc_fix,
                max_correspondence_distance=max(0.5, delta_alpha * 1.2),
                init=se2_to_se3(self.T_map_odom),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            H = reg.transformation  # 4x4 SE(3)

        except Exception as e:
            self.get_logger().warn(f"ICP failed: {e}")
            icp_success = False
            H = np.eye(4)

        # ---------------- convert SE3 → SE2 ----------------
        T_icp_se2 = se3_to_se2(H)

        # ---------------- extract motion ----------------
        dx = T_icp_se2[0, 2]
        dy = T_icp_se2[1, 2]
        trans_err = np.sqrt(dx**2 + dy**2)
        dtheta = np.arctan2(T_icp_se2[1, 0], T_icp_se2[0, 0])

        # ---------------- apply update ----------------
        maybe_T_map_odom = T_icp_se2 @ self.T_map_odom
        good = icp_success

        # reject large jumps
        if trans_err > 1.0:
            self.get_logger().warn(f"ICP rejected (too large motion): {trans_err:.2f}m")
            good = False
            
        if abs(dtheta) > max_angle_error:
            self.get_logger().warn(f"ICP rejected (too large rotation: {dtheta:.2f} rad)")
            good = False

        if good:
            self.T_map_odom = maybe_T_map_odom
            self.get_logger().info(
                f"ICP accepted: Δx={dx:.2f}, Δy={dy:.2f}, err={trans_err:.2f}"
            )

        self.get_logger().info(f"Map size: {len(self.map_points)}")
        
    def voxel_downsample(self, voxel_size=0.05):
        """
        Downsample 2D points using a voxel grid.

        voxel_size: size of each grid cell in meters
        """
        # compute voxel indices
        if self.map_points is None or len(self.map_points) == 0:
            return

        points = self.map_points

        # compute voxel indices
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # keep first point per voxel
        voxel_dict = {}

        for i, idx in enumerate(voxel_indices):
            key = (idx[0], idx[1])
            if key not in voxel_dict:
                voxel_dict[key] = points[i]

        self.map_points = np.array(list(voxel_dict.values()))

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