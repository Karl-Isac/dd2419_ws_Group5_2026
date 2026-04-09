import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles

class ScriptedDrive(Node):
    def __init__(self):
        super().__init__('scripted_drive')

        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)
        self.timer = self.create_timer(0.1, self.run)

        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def run(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        msg = DutyCycles()

        # Example trajectory:
        if t < 3.0:
            # drive forward
            msg.linear.x = 0.2

        elif t < 5.0:
            # rotate
            msg.angular.z = 0.5

        elif t < 8.0:
            # forward again
            msg.linear.x = 0.2

        else:
            # stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Finished trajectory")
            self.destroy_node()
            return

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScriptedDrive()
    rclpy.spin(node)

if __name__ == '__main__':
    main()