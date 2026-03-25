
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.pub = self.create_publisher(Path, "/plan", 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(5):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(i)
            ps.pose.position.y = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)
        self.get_logger().info(f"Published plan with {len(path.poses)} poses")

def main():
    rclpy.init()
    rclpy.spin(Planner())
    rclpy.shutdown()
