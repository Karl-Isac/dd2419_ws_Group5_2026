import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robp_interfaces.msg import DutyCycles


class TankTeleop(Node):
    def __init__(self):
        super().__init__('tank_teleop')

        self.pub = self.create_publisher(
            DutyCycles,
            '/phidgets/motor/duty_cycles',
            10
        )

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.max_speed = 0.8
        self.deadzone = 0.05

    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def joy_callback(self, joy_msg):
        msg = DutyCycles()

        # Typical Logitech F710 (XInput mode):
        # axes[1] = left stick vertical
        # axes[4] = right stick vertical

        left_input = -joy_msg.axes[3]   # invert so forward = positive
        right_input = -joy_msg.axes[4]

        left_input = self.apply_deadzone(left_input)
        right_input = self.apply_deadzone(right_input)

        msg.duty_cycle_left = left_input * self.max_speed
        msg.duty_cycle_right = right_input * self.max_speed

        # A button = stop
        if joy_msg.buttons[0] == 1:
            msg.duty_cycle_left = 0.0
            msg.duty_cycle_right = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TankTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()