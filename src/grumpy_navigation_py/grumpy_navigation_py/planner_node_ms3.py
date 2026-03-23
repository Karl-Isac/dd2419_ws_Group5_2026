
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from grumpy_interfaces import Goal

import yaml 


class PlannerNode(Node):
    def __init__(self):
        super().__init__("simple_planner_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("num_points", 40)

        self.declare_parameter("workspace_file", "workspace.yaml")  

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.num_points = int(self.get_parameter("num_points").value)

        self.workspace_poly = None
        try:
            path = self.get_parameter("workspace_file").value
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            self.workspace_poly = [tuple(p) for p in data["workspace"]["perimeter"]]
            if len(self.workspace_poly) < 3:
                self.get_logger().warn("workspace perimeter has < 3 points; disabling boundary check")
                self.workspace_poly = None
        except Exception as e:
            self.get_logger().warn(f"Could not load workspace_file; disabling boundary check: {e}")
            self.workspace_poly = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS I/O
        # self.goal_sub = self.create_subscription(PoseStamped, "/nav/goal", self.on_goal, 10)
        self.goal_sub = self.create_subscription(Goal, "/nav/goal", self.on_goal, 10)
        self.path_pub = self.create_publisher(Path, "/nav/path", 10)

        self.get_logger().info(
            f"PlannerNode up. world_frame={self.world_frame} base_frame={self.base_frame} "
            f"Sub: /nav/goal  Pub: /nav/path"
        )

    # Point-in-polygon (ray casting), works for concave polygons too  <-- NEW
    def inside_poly(self, x: float, y: float) -> bool:
        poly = self.workspace_poly
        if poly is None:
            return True  # boundary check disabled
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersects = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            if intersects:
                inside = not inside
            j = i
        return inside

    def get_pose_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),  # latest available
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().info(f"transform lookup error: {e}", throttle_duration_sec=1.0)
            return None

        t = tf.transform.translation
        return (t.x, t.y)

    def on_goal(self, goal_msg: Goal):
        # Transform goal into world_frame if needed
        goal_in_world = goal_msg.pose
        if goal_msg.header.frame_id and goal_msg.header.frame_id != self.world_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    goal_msg.header.frame_id,
                    rclpy.time.Time(),  # latest available
                    timeout=Duration(seconds=0.2),
                )
                goal_in_world = do_transform_pose(goal_msg, tf)
            except Exception as e:
                self.get_logger().warn(f"Could not transform goal to {self.world_frame}: {e}")
                return

        pose = self.get_pose_xy()
        if pose is None:
            return
        sx, sy = pose

        gx = goal_in_world.pose.position.x
        gy = goal_in_world.pose.position.y

        n = max(2, self.num_points)
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.world_frame

        # Build path and verify every point is inside polygon  <-- NEW
        for i in range(n):
            t = i / float(n - 1)
            x = sx + t * (gx - sx)
            y = sy + t * (gy - sy)

            if not self.inside_poly(x, y):
                self.get_logger().warn(
                    f"Path leaves workspace at i={i}/{n-1} point=({x:.2f},{y:.2f}); not publishing path"
                )
                return

            p = PoseStamped()
            p.header.stamp = now
            p.header.frame_id = self.world_frame
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        self.path_pub.publish(path)
        self.get_logger().info(f"Published path ({n} pts) to ({gx:.2f}, {gy:.2f})")


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
