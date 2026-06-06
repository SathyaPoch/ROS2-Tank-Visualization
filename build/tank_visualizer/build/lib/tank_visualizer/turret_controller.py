#!/usr/bin/env python3
"""
Turret Controller Node
Controls the tank turret rotation (yaw) and cannon elevation (pitch)
Publishes commands to:
  - /turret/yaw_command (Float32): Turret rotation angle in radians
  - /turret/pitch_command (Float32): Cannon elevation angle in radians
"""

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
        
        # State
        self.turret_yaw = 0.0  # Current yaw angle
        self.cannon_pitch = 0.0  # Current pitch angle
        
        # Rotation speeds
        self.yaw_speed = 0.1  # radians per update
        self.pitch_speed = 0.05  # radians per update
        
        # Timer to update at 20Hz
        self.timer = self.create_timer(0.05, self.update_turret)
        
        self.get_logger().info('Turret Controller Started!')
        self.print_controls()
    
    def print_controls(self):
        self.get_logger().info("""
        ======== TURRET CONTROLS ========
        Commands (send via ROS topics or modify this node):
        
        Turret Rotation (Yaw):
          - Use arrow keys LEFT/RIGHT to rotate turret
          - Or publish Float32 to /turret/yaw_command
        
        Cannon Elevation (Pitch):
          - Use arrow keys UP/DOWN to aim cannon
          - Or publish Float32 to /turret/pitch_command
        
        Example ROS commands:
          ros2 topic pub /turret/yaw_command std_msgs/Float32 "data: 1.57"
          ros2 topic pub /turret/pitch_command std_msgs/Float32 "data: 0.5"
        
        Current values (radians):
          Yaw: 0.0 (0 degrees)
          Pitch: 0.0 (0 degrees)
        ================================
        """)
    
    def update_turret(self):
        """Update and publish turret position"""
        # Publish current angles
        yaw_msg = Float32(data=self.turret_yaw)
        pitch_msg = Float32(data=self.cannon_pitch)
        
        self.yaw_pub.publish(yaw_msg)
        self.pitch_pub.publish(pitch_msg)
        
        # Simple demo: slowly rotate turret
        self.turret_yaw += 0.02
        if self.turret_yaw > 2 * math.pi:
            self.turret_yaw -= 2 * math.pi
        
        # Cannon goes up and down
        self.cannon_pitch = 0.3 * math.sin(self.get_clock().now().nanoseconds / 1e9)


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
