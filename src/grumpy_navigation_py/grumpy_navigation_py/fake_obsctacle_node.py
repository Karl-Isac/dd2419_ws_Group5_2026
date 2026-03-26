#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeObstacleNode(Node):
    def __init__(self):
        super().__init__("fake_obstacle_node")

        # Parameters
        self.declare_parameter("csv_file", "obstacles.csv")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 1.0)

        self.csv_file = self.get_parameter("csv_file").value
        self.frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("publish_rate").value

        # Publisher
        self.pub = self.create_publisher(PoseStamped, "/fake_obstacles", 10)

        # Load obstacles
        self.obstacles = self.load_csv(self.csv_file)
        self.get_logger().info(f"Loaded {len(self.obstacles)} obstacles")

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_obstacles)

    def load_csv(self, path):
        obstacles = []
        try:
            with open(path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    x = float(row["x"])
                    y = float(row["y"])
                    obstacles.append((x, y))
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")
        return obstacles

    def publish_obstacles(self):
        now = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(self.obstacles):
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0

            msg.pose.orientation.w = 1.0  # no rotation

            self.pub.publish(msg)

        self.get_logger().info("Published obstacles")


def main(args=None):
    rclpy.init(args=args)
    node = FakeObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
