import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import sys

class TurretController(Node):
    def __init__(self):
        super().__init__('turret_controller')
        
        # Publishers
        self.yaw_pub = self.create_publisher(Float32, '/turret/yaw_command', 10)
        self.pitch_pub = self.create_publisher(Float32, '/turret/pitch_command', 10)
    
        self.turret_yaw = 0.0  
        self.cannon_pitch = 0.0  
        
        self.yaw_speed = 0.1  
        self.pitch_speed = 0.05  
        
        self.timer = self.create_timer(0.05, self.update_turret)
        
        self.get_logger().info('Turret Controller Started!')
    
    def update_turret(self):
        """Update and publish turret position"""
        yaw_msg = Float32(data=self.turret_yaw)
        pitch_msg = Float32(data=self.cannon_pitch)
        
        self.yaw_pub.publish(yaw_msg)
        self.pitch_pub.publish(pitch_msg)
        
def main(args=None):
    rclpy.init(args=args)
    controller = TurretController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Turret Controller shutting down')
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
