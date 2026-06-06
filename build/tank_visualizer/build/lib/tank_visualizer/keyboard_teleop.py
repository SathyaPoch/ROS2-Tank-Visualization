import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading

# Key bindings
KEY_BINDINGS = {
    'w': (1.0,  0.0),   # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0,  1.0),   # Turn left
    'd': (0.0, -1.0),   # Turn right
}

LINEAR_SPEED  = 1.0   # m/s  — tweak as needed
ANGULAR_SPEED = 1.5   # rad/s — tweak as needed

def get_key(settings):
    """Read a single keypress without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            '\n--- Tank Keyboard Teleop ---\n'
            '  W = Forward\n'
            '  S = Backward\n'
            '  A = Turn Left\n'
            '  D = Turn Right\n'
            '  Any other key = STOP\n'
            '  Ctrl+C to quit\n'
        )

        # Run key-reading loop in a background thread so ROS can spin freely
        self._thread = threading.Thread(target=self.key_loop, daemon=True)
        self._thread.start()

    def key_loop(self):
        try:
            while rclpy.ok():
                key = get_key(self.settings)

                twist = Twist()

                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    twist.linear.x  = lin * LINEAR_SPEED
                    twist.angular.z = ang * ANGULAR_SPEED
                    self.get_logger().info(f"Key '{key.upper()}' → linear={twist.linear.x}, angular={twist.angular.z}")
                elif key == '\x03':          # Ctrl+C
                    break
                else:
                    # Any unknown key stops the tank
                    self.get_logger().info('STOP')

                self.publisher.publish(twist)

        finally:
            # Always send a stop on exit
            self.publisher.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()