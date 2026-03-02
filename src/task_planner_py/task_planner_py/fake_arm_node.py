import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FakeArmNode(Node):
    def __init__(self):
        super().__init__("fake_arm_node")

        self.subscription = self.create_subscription(
            String,
            "/arm/cmd",
            self.cmd_callback,
            10
        )

        self.done_pick_publisher = self.create_publisher(
            Bool,
            "/arm/done_pick",
            10
        )

    def cmd_callback(self, msg):
        self.get_logger().info("Received arm command. Waiting 5 seconds...")

        # Create a one-shot timer (5 seconds)
        self.timer = self.create_timer(5.0, self.delay)

    def delay(self):
        self.get_logger().info("5 seconds passed. Publishing done_pick.")

        msg = Bool()
        msg.data = True
        self.done_pick_publisher.publish(msg)

        # Destroy timer so it doesn't repeat
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = FakeArmNode()
    rclpy.spin(node)
    rclpy.shutdown()
