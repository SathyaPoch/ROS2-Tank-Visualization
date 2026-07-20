import rclpy
from rclpy.node import Node
from tank_interfaces.msg import TankState


class TankStateSubscriber(Node):
    """Subscribes to /tank_state and logs the custom TankState message."""

    def __init__(self):
        super().__init__('tank_state_subscriber')

        self.create_subscription(
            TankState,
            '/tank_state',
            self.state_callback,
            10,
        )

        self.get_logger().info('Tank State Subscriber started — listening on /tank_state')

    def state_callback(self, msg):
        self.get_logger().info(
            f'[{msg.tank_name}] '
            f'pos=({msg.x:.2f}, {msg.y:.2f}) '
            f'θ={msg.theta:.2f} '
            f'turret={msg.turret_yaw:.2f} '
            f'HP={msg.health:.1f}/{msg.max_health:.1f} '
            f'alive={msg.is_alive}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TankStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
