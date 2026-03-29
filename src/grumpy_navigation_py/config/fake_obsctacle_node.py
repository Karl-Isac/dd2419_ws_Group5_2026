#!/usr/bin/env python3

import csv
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeObstacleNode(Node):
    def __init__(self):
        super().__init__("fake_obstacle_node")

        self.frame_id = "map"
        self.csv_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "fake_obstacles.csv",
        )

        self.pub = self.create_publisher(PoseStamped, "/fake_obstacles", 10)

        self.obstacles = self.load_csv(self.csv_file)
        self.get_logger().info(f"Loaded {len(self.obstacles)} obstacles")

        self.publish_obstacles_once()

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

    def publish_obstacles_once(self):
        now = self.get_clock().now().to_msg()

        for x, y in self.obstacles:
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0

            self.pub.publish(msg)

        self.get_logger().info("Published obstacles once")


def main(args=None):
    rclpy.init(args=args)
    node = FakeObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
