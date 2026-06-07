import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading
from std_msgs.msg import Float32


KEY_BINDINGS = {
    'w': (1.0,  0.0),   
    's': (-1.0, 0.0),   
    'a': (0.0,  1.0),   
    'd': (0.0, -1.0),   
}

TURRET_SPEED = 0.1
LINEAR_SPEED  = 1.0   
ANGULAR_SPEED = 1.5  

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

        self.turret_pub = self.create_publisher(Float32, '/turret/yaw_command', 10)
        self.turret_yaw = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
        '\n--- Tank Keyboard Teleop ---\n'
        '  W = Forward\n'
        '  S = Backward\n'
        '  A = Turn Left\n'
        '  D = Turn Right\n'
        '  J = Turret Left\n'
        '  L = Turret Right\n'
        '  Any other key = STOP\n'
        '  Ctrl+C to quit\n'
        )

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
                elif key == 'j':
                    self.turret_yaw -= TURRET_SPEED
                    self.turret_pub.publish(Float32(data=self.turret_yaw))
                    self.get_logger().info(f"Turret Left → yaw={self.turret_yaw:.2f}")
                elif key == 'l':
                    self.turret_yaw += TURRET_SPEED
                    self.turret_pub.publish(Float32(data=self.turret_yaw))
                    self.get_logger().info(f"Turret Right → yaw={self.turret_yaw:.2f}")
                elif key == '\x03':
                    break
                else:
                    self.get_logger().info('STOP')
                self.publisher.publish(twist)

        finally:
            
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