import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty
from robp_interfaces.msg import DutyCycles

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(DutyCycles, '/phidgets/motor/duty_cycles', 10)

        

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key
    
    def run(self):
        msg = DutyCycles()
        while rclpy.ok():
            key = self.get_key()
            print(f'Key pressed: {key}')
            if key == 'w':
                msg.duty_cycle_left = 0.3
                msg.duty_cycle_right = 0.3
            elif key == 's':
                msg.duty_cycle_left = -0.3
                msg.duty_cycle_right = -0.3
            elif key == 'a':
                msg.duty_cycle_left = -0.15
                msg.duty_cycle_right = 0.15
            elif key == 'd':
                msg.duty_cycle_left = 0.15
                msg.duty_cycle_right = -0.15
                
            # if key == 'i':
            #     msg.duty_cycle_left = 0.5
            #     msg.duty_cycle_right = 0.25
            # elif key == 'j':
            #     msg.duty_cycle_left = -0.25
            #     msg.duty_cycle_right = -0.5
            # elif key == 'k':
            #     msg.duty_cycle_left = -0.6
            #     msg.duty_cycle_right = 0.6
            # elif key == 'l':
            #     msg.duty_cycle_left = 0.6
            #     msg.duty_cycle_right = -0.6
            
            
            
            elif key == 'c' or key == '\x03':
                msg.duty_cycle_left = 0.0
                msg.duty_cycle_right = 0.0
                self.pub.publish(msg)
                break
            else:
                msg.duty_cycle_left = 0.0
                msg.duty_cycle_right = 0.0

            self.pub.publish(msg)
def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt as kx:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# comment to test Git