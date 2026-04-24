
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
import math

class Object:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__("obstacle_detection_node")

        self.world_frame = "map"

        self.known_objects = []

        self.create_subscription(LaserScan, "/lidar/scan", self.on_laser_scan, 10)
        self.objects_pub = self.create_publisher(PoseStamped, "/detected_obstacles", 10)

    
    def on_laser_scan(self, msg):
        points = []

        for i, r in enumerate(msg.ranges):

            # --- filter invalid ---
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            # --- compute angle ---
            angle = msg.angle_min + i * msg.angle_increment

            # --- polar → cartesian ---
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y))

        # simple debug
        self.get_logger().info(f"Got {len(points)} valid points")


        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
