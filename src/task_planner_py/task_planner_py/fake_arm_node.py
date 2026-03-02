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
            String,
            "/arm/report_back",
            10
        )

    def cmd_callback(self, msg):
        self.get_logger().info("Received arm command. Waiting 5 seconds...")

        self.get_logger().info(f"msg.data: {msg.data}")

        # Create a one-shot timer (5 seconds)
        if msg.data == "pick":
            self.timer = self.create_timer(5.0, self.delay_pick)
        elif msg.data == "place":
            self.timer = self.create_timer(5.0, self.delay_place)


    def delay_pick(self):
        self.get_logger().info("5 seconds passed. Publishing done_pick.")
        self.get_logger().info("delay pick")

        msg = String()
        msg.data = "pick_success"
        self.done_pick_publisher.publish(msg)
        self.get_logger().info("delay_pick sen pick success")

        # Destroy timer so it doesn't repeat
        self.timer.cancel()

    def delay_place(self):
        self.get_logger().info("5 seconds passed. Publishing done_pick.")
        self.get_logger().info("delay place")

        msg = String()
        msg.data = "place_success"
        self.done_pick_publisher.publish(msg)

        # Destroy timer so it doesn't repeat
        self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = FakeArmNode()
    rclpy.spin(node)
    rclpy.shutdown()
