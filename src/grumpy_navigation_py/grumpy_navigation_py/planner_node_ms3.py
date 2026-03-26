#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")

        self.declare_parameter("world_frame", "map")
        self.world_frame = self.get_parameter("world_frame").value

        self.goal_sub = self.create_subscription(PoseStamped, "/nav/goal", self.on_goal, 10)
        self.path_pub = self.create_publisher(Path, "/nav/path_from_planner", 10)

        self.get_logger().info("AStarPlannerNode started")

    def get_start(self):
        # TODO: replace with TF lookup
        return (0.0, 0.0)

    def build_occupancy_grid(self):
        # TODO: fill from obstacles
        return None

    def run_astar(self, start, goal, grid):
        # TODO: implement A*
        return []

    def grid_path_to_ros_path(self, cells):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame

        for x, y in cells:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        return path

    def on_goal(self, goal_msg: PoseStamped):
        start = self.get_start()
        goal = (
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
        )

        grid = self.build_occupancy_grid()
        cells = self.run_astar(start, goal, grid)
        path = self.grid_path_to_ros_path(cells)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
