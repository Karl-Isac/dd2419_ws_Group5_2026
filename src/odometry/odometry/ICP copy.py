#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from pathlib import Path

import numpy as np
import rclpy.time
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

import tf_transformations
import tf2_ros
#from sklearn.neighbors import NearestNeighbors        maybe remove if not used, but for now we keep it since it's needed for ICP
from simpleicp import SimpleICP, PointCloud


base_dir = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent.parent
KNOWN_PATH = base_dir / 'Workspace/map_1_1.csv'


# ---------------- ICP ----------------

def scan_to_points(scan):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    points = []

    for r, a in zip(scan.ranges, angles):
        if np.isfinite(r):
            points.append([r * np.cos(a), r * np.sin(a)])

    return np.array(points)


#def icp(source, target, max_iter=20):
#    T = np.eye(3)
#
#    for _ in range(max_iter):
#        nbrs = NearestNeighbors(n_neighbors=1).fit(target)
#        _, indices = nbrs.kneighbors(source)
#        matched = target[indices[:, 0]]
#
#        mu_s = np.mean(source, axis=0)
#        mu_t = np.mean(matched, axis=0)
#
#        S = source - mu_s
#        M = matched - mu_t
#
#        W = M.T @ S
#        U, _, VT = np.linalg.svd(W)
#        R = U @ VT
#        t = mu_t - R @ mu_s
#
#        source = (R @ source.T).T + t
#
#        T_step = np.eye(3)
#        T_step[:2, :2] = R
#        T_step[:2, 2] = t
#
#        T = T_step @ T
#
#    return T


# ---------------- NODE ----------------

class ICPMapper(Node):

    def __init__(self):
        super().__init__('icp_mapper')

        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.scan_cb, 10)

        self.cmd_sub = self.create_subscription(
            String, '/command', self.cmd_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

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
        self.last_odom_pose = None

        self.publish_tf()

    # ---------------- callbacks ----------------

    def scan_cb(self, msg):
        self.last_scan = scan_to_points(msg)
        self.last_stamp = msg.header.stamp

    def cmd_cb(self, msg):
        if msg.data == "start":
            self.start_mapping()
        elif msg.data == "correct":
            self.correct_pose()

    # ---------------- mapping ----------------

    def start_mapping(self):
        if self.last_scan is None:
            self.get_logger().warn("No scan yet")
            return

        self.map_points = self.last_scan.copy()
        self.get_logger().info("Map initialized")

    # ---------------- ICP correction ----------------

#    def correct_pose(self):
#        if self.map_points is None or self.last_scan is None:
#            return
#
#        source = self.last_scan.copy()
#        target = self.map_points.copy()
#
#        simpleicp = SimpleICP(source, target)
#        T_icp = simpleicp.get_transformation()
#
#        # ---------------- STABILITY CHECK ----------------
#
#        dx = T_icp[0, 2]
#        dy = T_icp[1, 2]
#        trans_err = np.sqrt(dx * dx + dy * dy)
#        
#        # ---------------- APPLY UPDATE ----------------
#
#        T_total = T_icp
#        maybe_T_map_odom = T_total @ self.T_map_odom
#        good = True
#
#        # reject large jumps (no tuning knobs beyond fixed sanity limit)
#        if trans_err > 0.3:
#            self.get_logger().warn("ICP rejected (too large motion)")
#            good = False
#
#    
#        # consistency check: ICP should not fully contradict odometry direction
#        if self.last_odom_pose is not None:
#            odom_dx = self.last_odom_pose[0]
#            odom_dy = self.last_odom_pose[1]
#
#            dot = odom_dx * dx + odom_dy * dy
#            if dot < 0:
#                self.get_logger().warn("ICP disagrees with odometry → ignored")
#                good = False
#
#        if good:
#            self.T_map_odom = maybe_T_map_odom
#            self.get_logger().info(f"ICP accepted: Δx={dx:.2f}m, Δy={dy:.2f}m, error={trans_err:.2f}m")
#        # ---------------- update map ----------------
#
#        R = T_total[:2, :2]
#        t = T_total[:2, 2]
#
#        transformed_scan = (R @ self.last_scan.T).T + t
#
#        if self.map_points is None:
#            self.map_points = transformed_scan
#        else:
#            self.map_points = np.vstack((self.map_points, transformed_scan))
#
#        self.get_logger().info(f"Map size: {len(self.map_points)}")

    def correct_pose(self):
        if self.map_points is None or self.last_scan is None:
            return

        # ---------------- prepare point clouds ----------------
        pc_fix = PointCloud(to_3d(self.map_points), columns=["x", "y", "z"])
        pc_mov = PointCloud(to_3d(self.last_scan), columns=["x", "y", "z"])
        
        self.get_logger().info(f"Map size: {len(pc_fix)}, Scan size: {len(pc_mov)}")

        icp = SimpleICP()
        icp.add_point_clouds(pc_fix, pc_mov)

        # ---------------- run ICP ----------------
        icp_success = True
        try:
            H, X_mov_transformed, _, _ = icp.run(max_overlap_distance=1.0, max_iterations=10)
            #H, X_mov_transformed, _ = icp.run(max_overlap_distance=0.5, max_iterations=50)
        except Exception as e:
            self.get_logger().warn(f"ICP failed: {e}")
            icp_success = False
            H = np.eye(4)
            X_mov_transformed = to_3d(self.last_scan)

        # H is 3x3 homogeneous transform
        T_icp_se2 = se3_to_se2(H)

        # ---------------- extract motion ----------------
        dx = T_icp_se2[0, 2]
        dy = T_icp_se2[1, 2]
        trans_err = np.sqrt(dx ** 2 + dy ** 2)

        # ---------------- apply update ----------------
        maybe_T_map_odom = T_icp_se2 @ self.T_map_odom
        good = icp_success

        # reject large jumps
        if trans_err > 0.3:
            self.get_logger().warn("ICP rejected (too large motion)")
            good = False

        # consistency with odometry
        if self.last_odom_pose is not None:
            odom_dx = self.last_odom_pose[0]
            odom_dy = self.last_odom_pose[1]

            dot = odom_dx * dx + odom_dy * dy
            if dot < 0:
                self.get_logger().warn("ICP disagrees with odometry → ignored")
                good = False

        if good:
            self.T_map_odom = maybe_T_map_odom

        # ---------------- update map ----------------
        transformed_scan = np.array(X_mov_transformed)[:, :2]

        if self.map_points is None:
            self.map_points = transformed_scan
        else:
            self.map_points = np.vstack((self.map_points, transformed_scan))

        self.get_logger().info(f"Map size: {len(self.map_points)}")

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
#def to_3d(points):
#    return np.hstack((points, np.zeros((points.shape[0], 1))))
def to_3d(points):
    return np.array(
        np.hstack((points, np.zeros((points.shape[0], 1)))),
        dtype=np.float64,
        copy=True
    )

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

def main(args=None):
    rclpy.init(args=args)
    node = ICPMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()