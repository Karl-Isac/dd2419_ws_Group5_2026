
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class SimplePlannerNode(Node):
    def __init__(self):
        super().__init__("simple_planner_node")

        self.declare_parameter("world_frame", "map")       # "map" or "odom"
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("num_points", 40)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.num_points = int(self.get_parameter("num_points").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(PoseStamped, "/nav/goal", self.on_goal, 10)
        self.path_pub = self.create_publisher(Path, "/nav/path", 10)

        self.get_logger().info("SimplePlannerNode up. Sub: /nav/goal  Pub: /nav/path")

    def get_pose_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f"transform lookup error: {e}", throttle_duration_sec=1.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def on_goal(self, goal_msg: PoseStamped):
        # Keep it simple: assume goal is already in world_frame
        if goal_msg.header.frame_id and goal_msg.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f"Goal frame '{goal_msg.header.frame_id}' != '{self.world_frame}'. "
                f"Publish goals in {self.world_frame} (or add TF transform here)."
            )
            return

        pose = self.get_pose_xy()
        if pose is None:
            return
        sx, sy = pose

        gx = goal_msg.pose.position.x
        gy = goal_msg.pose.position.y

        n = max(2, self.num_points)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame

        for i in range(n):
            t = i / float(n - 1)
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = sx + t * (gx - sx)
            p.pose.position.y = sy + t * (gy - sy)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            path.poses.append(p)

        self.path_pub.publish(path)
        self.get_logger().info(f"Published path ({n} pts) to ({gx:.2f}, {gy:.2f})")


def main():
    rclpy.init()
    node = SimplePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
