#!/usr/bin/env python3
import os
import math
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import Buffer, TransformListener, TransformException

from ament_index_python.packages import get_package_share_directory
from shapely.geometry import Point, Polygon


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__("obstacle_detection_node")

        self.world_frame = "map"
        self.workspace_file = "fake_workspace.yaml"
        self.workspace_package = "grumpy_navigation_py"

        self.cluster_distance = 0.08
        self.min_cluster_size = 2

        self.workspace_polygon = self.load_workspace_polygon()

        self.create_subscription(LaserScan, "/lidar/scan", self.on_laser_scan, 10)
        self.obstacles_pub = self.create_publisher(PoseArray, "/detected_obstacles", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def load_workspace_polygon(self):
        pkg_share = get_package_share_directory(self.workspace_package)
        path = os.path.join(pkg_share, "config", self.workspace_file)

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        points = data["workspace"]["perimeter"]
        return Polygon(points)

    def in_workspace(self, x, y):
        return self.workspace_polygon.contains(Point(x, y))

    def cluster_points(self, points):
        clusters = []
        used = [False] * len(points)

        for i in range(len(points)):
            if used[i]:
                continue

            cluster = [points[i]]
            used[i] = True

            changed = True
            while changed:
                changed = False

                for j in range(len(points)):
                    if used[j]:
                        continue

                    xj, yj = points[j]

                    for x, y in cluster:
                        if math.hypot(xj - x, yj - y) <= self.cluster_distance:
                            cluster.append(points[j])
                            used[j] = True
                            changed = True
                            break

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def on_laser_scan(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as e:
            # self.get_logger().warn(f"TF failed: {e}")
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = yaw_from_quat(tf.transform.rotation)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            mx = tx + cos_yaw * lx - sin_yaw * ly
            my = ty + sin_yaw * lx + cos_yaw * ly

            if self.in_workspace(mx, my):
                points.append((mx, my))

        clusters = self.cluster_points(points)

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame

        for cluster in clusters:
            for x, y in cluster:
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation.w = 1.0
                out.poses.append(pose)

        self.obstacles_pub.publish(out)

        # self.get_logger().info(
        #     f"points={len(points)}, clusters={len(clusters)}, published={len(out.poses)}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
