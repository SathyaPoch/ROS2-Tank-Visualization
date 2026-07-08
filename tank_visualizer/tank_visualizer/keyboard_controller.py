import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty
import sys
import tty
import termios
import threading

KEY_BINDINGS = {
    'w': (1.0,  0.0),
    's': (-1.0, 0.0),
    'a': (0.0,  1.0),
    'd': (0.0, -1.0),
}

LINEAR_SPEED  = 1.0
ANGULAR_SPEED = 1.5
TURRET_SPEED  = 0.05  # radians per timer tick (20Hz = smooth)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher   = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.turret_pub  = self.create_publisher(Float32, '/turret/yaw_command', 10)
        self.fire_pub    = self.create_publisher(Empty,  '/tank/fire', 10)  
        self.fire_mg_pub = self.create_publisher(Empty,  '/tank/fire_mg', 10) 

        self.turret_yaw     = 0.0
        self.turret_yaw_vel = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

        # Timer runs at 20Hz — smoothly integrates turret velocity
        self.create_timer(0.05, self.turret_timer)

        self.get_logger().info(
            '\n--- Tank Keyboard Teleop ---\n'
            '  W = Forward\n'
            '  S = Backward\n'
            '  A = Turn Left\n'
            '  D = Turn Right\n'
            '  J = Turret Left\n'
            '  L = Turret Right\n'
            '  SPACE = FIRE CANNON\n'
            '  K = FIRE MACHINE GUN\n'
            '  Ctrl+C to quit\n'
        )

        self._thread = threading.Thread(target=self.key_loop, daemon=True)
        self._thread.start()

    def turret_timer(self):
        """Runs at 20Hz — smoothly moves turret while key is held"""
        if self.turret_yaw_vel != 0.0:
            self.turret_yaw += self.turret_yaw_vel
            self.turret_pub.publish(Float32(data=self.turret_yaw))

    def key_loop(self):
        try:
            while rclpy.ok():
                key = get_key(self.settings)
                twist = Twist()

                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    twist.linear.x  = lin * LINEAR_SPEED
                    twist.angular.z = ang * ANGULAR_SPEED
                    self.turret_yaw_vel = 0.0  # stop turret when moving tank
                elif key == 'j':
                    self.turret_yaw_vel = -TURRET_SPEED
                elif key == 'l':
                    self.turret_yaw_vel = TURRET_SPEED
                elif key == ' ':                 
                    self.fire_pub.publish(Empty())
                    self.get_logger().info('FIRE!')
                elif key == 'k':
                    self.fire_mg_pub.publish(Empty())
                    self.get_logger().info('MG — FIRE!')
                elif key == '\x03':
                    break
                else:
                    self.turret_yaw_vel = 0.0
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