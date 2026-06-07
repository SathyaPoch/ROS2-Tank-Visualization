import rclpy
from rclpy.node import Node
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from std_msgs.msg import Float32
import tf2_ros

def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class TankBodyVisualizer(Node):
    def __init__(self):
        super().__init__('tank_body_visualizer')

        # Publisher and Subscriber
        self.marker_array_pub = self.create_publisher(MarkerArray, '/tank_body_markers', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.turret_yaw = 0.0
        self.cannon_pitch = 0.0

        self.create_subscription(Float32, '/turret/yaw_command', self.turret_yaw_callback, 10)
        self.create_subscription(Float32, '/turret/pitch_command', self.cannon_pitch_callback, 10)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.body_length = 1.0
        self.body_width = 0.6
        self.body_height = 0.3
        self.wheel_radius = 0.15
        self.wheel_width = 0.1

        self.turret_radius = 0.25
        self.turret_height = 0.2
        self.cannon_length = 0.6
        self.cannon_radius = 0.05

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Tank Body Visualizer Node Started')

    def turret_yaw_callback(self, msg):
        self.turret_yaw = msg.data

    def cannon_pitch_callback(self, msg):
        self.cannon_pitch = max(-0.785, min(0.785, msg.data))

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x    
        self.angular_velocity = msg.angular.z  

    def timer_callback(self):
        time_now = self.get_clock().now().to_msg()

        dt = 0.05 
        self.theta += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt

        t = TransformStamped()
        t.header.stamp = time_now
        t.header.frame_id = 'map'
        t.child_frame_id = 'tank_body'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.15
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        t_turret = TransformStamped()
        t_turret.header.stamp = time_now
        t_turret.header.frame_id = 'tank_body'
        t_turret.child_frame_id = 'tank_turret'
        t_turret.transform.translation.z = self.body_height / 2.0
        t_turret.transform.rotation.z = math.sin(self.turret_yaw / 2.0)
        t_turret.transform.rotation.w = math.cos(self.turret_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t_turret)

        marker_array = MarkerArray()

        chassis = Marker()
        chassis.header.stamp = time_now
        chassis.header.frame_id = 'tank_body'
        chassis.id = 0
        chassis.type = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.scale.x = self.body_length
        chassis.scale.y = self.body_width
        chassis.scale.z = self.body_height
        chassis.color.r = 0.0
        chassis.color.g = 0.5
        chassis.color.b = 0.0
        chassis.color.a = 1.0
        marker_array.markers.append(chassis)

        def add_wheel(marker_id, x_offset, y_side_multiplier):
            wheel = Marker()
            wheel.header.stamp = time_now
            wheel.header.frame_id = 'tank_body'
            wheel.id = marker_id
            wheel.type = Marker.CYLINDER
            wheel.action = Marker.ADD
            wheel.scale.x = self.wheel_radius * 2
            wheel.scale.y = self.wheel_radius * 2
            wheel.scale.z = self.wheel_width
            wheel.color.r = 0.1
            wheel.color.g = 0.1
            wheel.color.b = 0.1
            wheel.color.a = 1.0
            wheel.pose.position.x = x_offset
            wheel.pose.position.y = (self.body_width / 2.0 + self.wheel_width / 2.0) * y_side_multiplier
            wheel.pose.position.z = -0.05
            wheel.pose.orientation = euler_to_quaternion(yaw=0, pitch=1.5708, roll=0)
            marker_array.markers.append(wheel)

        x_positions = [-0.3, 0.0, 0.3]
        wheel_id = 1
        for x_pos in x_positions:
            add_wheel(wheel_id, x_pos, 1)  
            wheel_id += 1
            add_wheel(wheel_id, x_pos, -1)  
            wheel_id += 1

        turret_base = Marker()
        turret_base.header.stamp = time_now
        turret_base.header.frame_id = 'tank_turret'
        turret_base.id = 100
        turret_base.type = Marker.CYLINDER
        turret_base.action = Marker.ADD
        turret_base.scale.x = self.turret_radius * 2
        turret_base.scale.y = self.turret_radius * 2
        turret_base.scale.z = self.turret_height
        turret_base.color.r = 0.2
        turret_base.color.g = 0.4
        turret_base.color.b = 0.2
        turret_base.color.a = 1.0
        turret_base.pose.position.z = self.turret_height / 2.0
        marker_array.markers.append(turret_base)

  
        cannon = Marker()
        cannon.header.stamp = time_now
        cannon.header.frame_id = 'tank_turret'
        cannon.id = 101
        cannon.type = Marker.CYLINDER
        cannon.action = Marker.ADD
        cannon.scale.x = self.cannon_radius * 2
        cannon.scale.y = self.cannon_radius * 2
        cannon.scale.z = self.cannon_length
        cannon.color.r = 0.1
        cannon.color.g = 0.1
        cannon.color.b = 0.1
        cannon.color.a = 1.0
        cannon.pose.position.x = (self.cannon_length / 2.0) * math.cos(self.cannon_pitch)
        cannon.pose.position.z = self.turret_height + (self.cannon_length / 2.0) * math.sin(self.cannon_pitch)
        cannon.pose.orientation = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + self.cannon_pitch, roll=0)
        marker_array.markers.append(cannon)

        self.marker_array_pub.publish(marker_array)
def main(args=None):
    rclpy.init(args=args)
    node = TankBodyVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()