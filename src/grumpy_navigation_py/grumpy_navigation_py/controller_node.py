
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.sub = self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_plan(self, msg: Path):
        cmd = Twist()
        cmd.linear.x = 0.2 if len(msg.poses) > 0 else 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        self.get_logger().info(f"Got plan ({len(msg.poses)} poses) -> publishing cmd_vel")

def main():
    rclpy.init()
    rclpy.spin(Controller())
    rclpy.shutdown()
