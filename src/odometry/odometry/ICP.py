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
This ICP corrects lidar-points when a good ICP match is found, and uses the corrected map for the next ICP, which makes it more robust to drift. (hopefully)
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
        self.raw_scan = None
        self.last_scan = None
        self.pre_localization_scan = None
        self.map_points = None
        self.pre_map_points = None
        self.last_stamp = None
        self.last_angular_velocity = 0
        
        self.temp_incrementer = 0

        self.publish_tf()

    # ---------------- callbacks ----------------

    #def scan_cb(self, msg):
    #    self.last_scan = scan_to_points(msg)
    #    self.last_stamp = msg.header.stamp
    #    if self.last_angular_velocity < 0.1: # only update when robot is not rotating fast, otherwise ICP will fail
    #        self.pre_map_points = np.vstack((self.pre_map_points, self.last_scan)) if self.pre_map_points is not None else self.last_scan
    #    if self.map_points is not None and len(self.map_points) > 20000:
    #        self.voxel_downsample()
    
    def scan_cb(self, msg):
        if abs(self.last_angular_velocity) > 0.1:
            return
        
        self.raw_scan = scan_to_points(msg)
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
        self.fix_scan()
        if self.last_scan is None:
            self.get_logger().warn("No scan yet")
            return
        self.map_points = np.vstack((self.map_points, self.last_scan)) if self.map_points is not None else self.last_scan
        self.get_logger().info("Map initialized or updated")

    # ---------------- ICP correction ----------------

    def correct_pose(self):
        self.fix_scan()
        if self.map_points is None or self.last_scan is None:
            return

        points_to_map = None
        if self.pre_localization_scan is not None:
            self.get_logger().info("Using pre-localization scan for ICP")
            points_to_map = self.pre_localization_scan
            self.pre_localization_scan = None
        else:
            self.get_logger().info("Using last scan for ICP")
            points_to_map = self.map_points
            
        # ---------------- convert to Open3D point clouds ----------------
        pc_fix = o3d.geometry.PointCloud()
        pc_fix.points = o3d.utility.Vector3dVector(to_3d(points_to_map))

#------------------------------test------------------------------
        R_map = self.T_map_odom[:2, :2]
        t_map = self.T_map_odom[:2, 2]

        scan_in_map = (R_map @ self.last_scan.T).T + t_map

#------------------------------test------------------------------
        pc_mov = o3d.geometry.PointCloud()
        pc_mov.points = o3d.utility.Vector3dVector(to_3d(scan_in_map))

        self.get_logger().info(
            f"Map size: {len(points_to_map)}, Scan size: {len(scan_in_map)}"
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
                max_correspondence_distance=max(0.3, delta_alpha * 1.2),
                init=np.eye(4), # no initial guess, since we are already in the map frame
                #init=se2_to_se3(self.T_map_odom),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            H = reg.transformation  # 4x4 SE(3)

        except Exception as e:
            self.get_logger().warn(f"ICP failed: {e}")
            return
        
        if reg.fitness < 0.8:
            self.get_logger().warn(f"ICP rejected (low fitness: {reg.fitness:.2f})")
            return
        self.temp_incrementer += 1
        self.export_map_to_csv(filename=f"scan_points_{self.temp_incrementer}.csv", points=self.last_scan) # export the map used for ICP, for visualization and debugging

        # ---------------- convert SE3 → SE2 ----------------
        T_icp_se2 = se3_to_se2(H)

        # ---------------- extract motion ----------------
        dx = T_icp_se2[0, 2]
        dy = T_icp_se2[1, 2]
        trans_err = np.sqrt(dx**2 + dy**2)
        dtheta = np.arctan2(T_icp_se2[1, 0], T_icp_se2[0, 0])

        # ---------------- apply update ----------------
        correction = np.linalg.inv(T_icp_se2) ##

        maybe_T_map_odom = correction @ self.T_map_odom
        #maybe_T_map_odom = T_icp_se2 @ self.T_map_odom

        ## reject large jumps
        #if trans_err > max(0.3, delta_len):
        #    self.get_logger().warn(f"ICP rejected (too large motion): {trans_err:.2f}m")
        #    icp_success = False
        #    
        #if abs(dtheta) > max_angle_error:
        #    self.get_logger().warn(f"ICP rejected (too large rotation: {dtheta:.2f} rad)")
        #    icp_success = False

        if icp_success:
            self.T_map_odom = maybe_T_map_odom
            self.get_logger().info(
                f"ICP accepted: Δx={dx:.2f}, Δy={dy:.2f}, err={trans_err:.2f}"
            )
        
#    def voxel_downsample(self, voxel_size=0.05):
#        """
#        Downsample 2D points using a voxel grid.
#
#        voxel_size: size of each grid cell in meters
#        """
#        # compute voxel indices
#        if self.map_points is None or len(self.map_points) == 0:
#            return
#
#        points = self.map_points
#
#        # compute voxel indices
#        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
#
#        # keep first point per voxel
#        voxel_dict = {}
#
#        for i, idx in enumerate(voxel_indices):
#            key = (idx[0], idx[1])
#            if key not in voxel_dict:
#                voxel_dict[key] = points[i]
#
#        self.map_points = np.array(list(voxel_dict.values()))
#
#    # ---------------- TF ----------------

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