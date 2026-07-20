import rclpy
from rclpy.node import Node
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32, Empty
from tank_interfaces.msg import TankState

from tank_visualizer.tank_game_logic import TankGameLogic


# ──────────────────────────────────────────────────────────────────────
# Quaternion helpers (these use geometry_msgs.Quaternion for RViz markers)
# ──────────────────────────────────────────────────────────────────────

def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def compose_quaternion(q_parent, q_child):
    """Multiply two quaternions (Quaternion msg objects): result = q_parent * q_child."""
    x1, y1, z1, w1 = q_parent.x, q_parent.y, q_parent.z, q_parent.w
    x2, y2, z2, w2 = q_child.x, q_child.y, q_child.z, q_child.w
    return Quaternion(
        x=w1*x2 + x1*w2 + y1*z2 - z1*y2,
        y=w1*y2 - x1*z2 + y1*w2 + z1*x2,
        z=w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w=w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate_vec_by_quat(x, y, z, q):
    """Rotate a local-frame vector (x, y, z) by quaternion q, return world-frame (x, y, z)."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # v' = v + 2*qw*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
    ux, uy, uz = qx, qy, qz
    # cross1 = u x v
    c1x = uy*z - uz*y
    c1y = uz*x - ux*z
    c1z = ux*y - uy*x
    # cross2 = u x cross1
    c2x = uy*c1z - uz*c1y
    c2y = uz*c1x - ux*c1z
    c2z = ux*c1y - uy*c1x
    rx = x + 2*qw*c1x + 2*c2x
    ry = y + 2*qw*c1y + 2*c2y
    rz = z + 2*qw*c1z + 2*c2z
    return rx, ry, rz


class TankBodyVisualizer(Node):
    """ROS2 node responsible for:
      - subscribing to control topics
      - delegating game-logic updates to TankGameLogic
      - building RViz Marker/MarkerArray visualizations from game state
    """

    def __init__(self):
        super().__init__('tank_body_visualizer')

        # ── Game logic (all state lives here) ──
        self.logic = TankGameLogic()

        # ── ROS2 pub / sub ──
        self.marker_array_pub = self.create_publisher(MarkerArray, '/tank_body_markers', 10)
        self.tank_state_pub   = self.create_publisher(TankState, '/tank_state', 10)

        self.create_subscription(Float32, '/turret/yaw_command',   self.turret_yaw_callback,   10)
        self.create_subscription(Float32, '/turret/pitch_command', self.cannon_pitch_callback,  10)
        self.create_subscription(Twist,   '/cmd_vel',              self.cmd_vel_callback,       10)
        self.create_subscription(Empty,   '/tank/fire',            self.fire_callback,          10)
        self.create_subscription(Empty,   '/tank/fire_mg',         self.fire_mg_callback,       10)

        # Number of timer ticks (at 20 Hz) during which a DELETEALL marker
        # is broadcast on startup. This force-clears any stale/ghost markers
        # left over in RViz from a previous version of this node (e.g. an
        # old third tank or extra marker that no longer exists in the code
        # but never received an explicit DELETE), without requiring the
        # user to manually hit "Reset" in RViz.
        self._startup_clear_ticks = 40   # ~2 seconds at 20 Hz

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Tank Body Visualizer Node Started')

    # ------------------------------------------------------------------
    # Subscription callbacks — thin wrappers that forward to logic
    # ------------------------------------------------------------------

    def turret_yaw_callback(self, msg):
        self.logic.set_turret_yaw(msg.data)

    def cannon_pitch_callback(self, msg):
        self.logic.set_cannon_pitch(msg.data)

    def cmd_vel_callback(self, msg):
        self.logic.set_velocity(msg.linear.x, msg.angular.z)

    def fire_callback(self, msg):
        self.logic.fire_cannon()

    def fire_mg_callback(self, msg):
        self.logic.fire_mg()

    # ------------------------------------------------------------------
    # Marker builders — projectiles + sparks
    # ------------------------------------------------------------------

    def build_projectile_markers(self, time_now):
        logic = self.logic
        markers   = []
        active_ids = set()

        for p in logic.projectiles:
            m = Marker()
            m.header.stamp    = time_now
            m.header.frame_id = 'map'
            m.ns     = 'projectiles'
            m.id     = p['id']
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 0.05   # black cannon shell
            m.color.g = 0.05
            m.color.b = 0.05
            m.color.a = 1.0
            m.pose.position.x  = p['x']
            m.pose.position.y  = p['y']
            m.pose.position.z  = p['z']
            m.pose.orientation.w = 1.0
            markers.append(m)
            active_ids.add(p['id'])

        for old_id in logic.prev_proj_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'projectiles'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        logic.prev_proj_ids = active_ids
        return markers

    def build_mg_projectile_markers(self, time_now):
        """Small silver/grey bullets from the machine gun."""
        logic = self.logic
        markers    = []
        active_ids = set()

        for p in logic.mg_projectiles:
            m = Marker()
            m.header.stamp    = time_now
            m.header.frame_id = 'map'
            m.ns     = 'mg_projectiles'
            m.id     = p['id']
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.04   # much smaller than cannon shell
            m.color.r = 0.8
            m.color.g = 0.8
            m.color.b = 0.85   # silver-grey
            m.color.a = 1.0
            m.pose.position.x  = p['x']
            m.pose.position.y  = p['y']
            m.pose.position.z  = p['z']
            m.pose.orientation.w = 1.0
            markers.append(m)
            active_ids.add(p['id'])

        for old_id in logic.prev_mg_proj_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'mg_projectiles'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        logic.prev_mg_proj_ids = active_ids
        return markers

    def build_spark_markers(self, time_now):
        """
        Each particle ages from white-hot → orange → red → transparent.

        Muzzle particles: smaller, blue-white hot core.
        Impact particles: larger burst, orange-yellow arc then fade red.

        frac = remaining_lifetime / total_lifetime  (1.0 → 0.0)
        """
        logic = self.logic
        markers    = []
        active_ids = set()

        for s in logic.sparks:
            frac = s['lifetime'] / s['total_lifetime']   # 1.0 → 0.0
            is_muzzle = s.get('is_muzzle', False)

            if is_muzzle:
                # White-hot → yellow → gone quickly
                size = max(0.02, 0.10 * frac)
                r    = 1.0
                g    = min(1.0, frac * 1.5)
                b    = max(0.0, frac * 2.0 - 0.5)
                a    = min(1.0, frac * 3.0)
            else:
                # Large orange burst → shrinking red ember → transparent
                size = max(0.02, 0.22 * frac)
                r    = 1.0
                g    = max(0.0, frac * 0.9 - 0.1)       # orange → red
                b    = max(0.0, frac * 2.5 - 1.8)       # slight white-hot at birth only
                a    = min(1.0, frac * 2.5)              # quick snap to opaque, slow fade

            m = Marker()
            m.header.stamp    = time_now
            m.header.frame_id = 'map'
            m.ns     = 'sparks'
            m.id     = s['id']
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            m.pose.position.x  = s['x']
            m.pose.position.y  = s['y']
            m.pose.position.z  = s['z']
            m.pose.orientation.w = 1.0
            markers.append(m)
            active_ids.add(s['id'])

        # DELETE markers for sparks that just expired
        for old_id in logic.prev_spark_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'sparks'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        logic.prev_spark_ids = active_ids
        return markers

    # ------------------------------------------------------------------
    # Health bar + game-over builders
    # ------------------------------------------------------------------
    def build_health_bar_markers(self, time_now):
        logic = self.logic
        markers = []

        def make_bar(x, y, theta, hp, max_hp, id_base):
            bar_length    = 0.7    # full-length of the gauge, along world X
            bar_depth     = 0.08
            bar_thickness = 0.08   # vertical thickness of the bar itself
            base_z        = 1.1    # fixed height above ground where the bar floats

            bg = Marker()
            bg.header.stamp    = time_now
            bg.header.frame_id = 'map'
            bg.ns     = 'health_bars'
            bg.id     = id_base
            bg.type   = Marker.CUBE
            bg.action = Marker.ADD
            bg.scale.x = bar_length
            bg.scale.y = bar_depth
            bg.scale.z = bar_thickness
            bg.color.r = 0.15
            bg.color.g = 0.15
            bg.color.b = 0.15
            bg.color.a = 0.85
            bg.pose.position.x = x
            bg.pose.position.y = y
            bg.pose.position.z = base_z
            bg.pose.orientation = euler_to_quaternion(yaw=theta, pitch=0, roll=0)
            markers.append(bg)

            frac = max(0.0, hp / float(max_hp))
            fill_length = max(0.01, bar_length * frac)

            fill = Marker()
            fill.header.stamp    = time_now
            fill.header.frame_id = 'map'
            fill.ns     = 'health_bars'
            fill.id     = id_base + 1
            fill.type   = Marker.CUBE
            fill.action = Marker.ADD
            fill.scale.x = fill_length
            fill.scale.y = bar_depth * 0.7
            fill.scale.z = bar_thickness * 0.7
            if frac > 0.5:
                fill.color.r, fill.color.g, fill.color.b = 0.1, 0.9, 0.1
            elif frac > 0.25:
                fill.color.r, fill.color.g, fill.color.b = 0.9, 0.9, 0.1
            else:
                fill.color.r, fill.color.g, fill.color.b = 0.9, 0.1, 0.1
            fill.color.a = 1.0
            # Anchor the fill to the "back" edge of the background bar (along
            # the tank's own heading) so it visibly depletes as HP drops,
            # instead of shrinking symmetrically from the center.
            offset = (bar_length - fill_length) / 2.0
            fill.pose.position.x = x - offset * math.cos(theta)
            fill.pose.position.y = y - offset * math.sin(theta)
            fill.pose.position.z = base_z + 0.002   # tiny lift to avoid z-fighting with bg
            fill.pose.orientation = euler_to_quaternion(yaw=theta, pitch=0, roll=0)
            markers.append(fill)

            # Offset the text to the left end of the bar so it co-rotates
            # with the bar as the tank turns.  The bar is bar_length wide and
            # aligned along theta, so shift by bar_length/2 in the +theta
            # direction to reach the leading edge.
            text_offset = bar_length / 2.0 + 0.08   # slightly beyond the bar edge
            text = Marker()
            text.header.stamp    = time_now
            text.header.frame_id = 'map'
            text.ns     = 'health_bars'
            text.id     = id_base + 2
            text.type   = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.18
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x = x + text_offset * math.cos(theta)
            text.pose.position.y = y + text_offset * math.sin(theta)
            text.pose.position.z = base_z + 0.22
            text.pose.orientation.w = 1.0
            text.text = 'HP: {:.1f}'.format(hp)
            markers.append(text)

        make_bar(logic.x,  logic.y,  logic.theta,  logic.hp1, logic.MAX_HP, 5000)
        make_bar(logic.x2, logic.y2, logic.theta2, logic.hp2, logic.MAX_HP, 5010)

        return markers

    def build_game_over_marker(self, time_now):
        logic = self.logic
        if not logic.game_over:
            return []

        text = Marker()
        text.header.stamp    = time_now
        text.header.frame_id = 'map'
        text.ns     = 'game_over'
        text.id     = 6000
        text.type   = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.8
        if logic.winner == 'tank1':
            text.text = 'YOU WIN!'
            text.color.r, text.color.g, text.color.b = 0.1, 1.0, 0.1
        else:
            text.text = 'YOU LOSE!'
            text.color.r, text.color.g, text.color.b = 1.0, 0.1, 0.1
        text.color.a = 1.0
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 2.5
        text.pose.orientation.w = 1.0
        return [text]

    # ------------------------------------------------------------------
    # Wall marker builder
    # ------------------------------------------------------------------

    def build_wall_markers(self, time_now):
        logic = self.logic
        markers = []
        for i, (cx, cy, length, thickness, yaw) in enumerate(logic.wall_segments):
            m = Marker()
            m.header.stamp    = time_now
            m.header.frame_id = 'map'
            m.ns     = 'walls'
            m.id     = 3000 + i
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = length
            m.scale.y = thickness
            m.scale.z = logic.wall_height
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 1.0
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = logic.wall_height / 2.0
            m.pose.orientation = euler_to_quaternion(yaw=yaw, pitch=0, roll=0)
            markers.append(m)
        return markers

    # ------------------------------------------------------------------
    # Tank marker builder — now takes a full WORLD pose and bakes it
    # directly into every marker instead of relying on a TF frame_id.
    # This removes the render-time TF lookup that was causing the
    # stutter / "ghost tank" duplication in RViz.
    # ------------------------------------------------------------------

    def build_tank_markers(self, time_now, body_x, body_y, body_theta,
                           turret_yaw, cannon_pitch, id_offset):
        """Build all markers for one tank, positioned directly in the 'map' frame.

        body_x, body_y, body_theta : world-space pose of the tank body
        turret_yaw                 : turret yaw relative to the body
        cannon_pitch                : cannon pitch relative to the turret
        id_offset                   : 0 for tank1, 300 for tank2
        """
        logic = self.logic
        markers = []

        ns = 'tank1' if id_offset == 0 else 'tank2'
        body_z = 0.15  # matches the old TF broadcast for tank_body -> map

        q_body = euler_to_quaternion(yaw=body_theta, pitch=0, roll=0)
        # Turret's world yaw = body yaw + turret yaw (turret frame sits body_height/2 above body origin)
        q_turret = euler_to_quaternion(yaw=body_theta + turret_yaw, pitch=0, roll=0)
        turret_world_z = body_z + logic.body_height / 2.0

        # Chassis
        chassis = Marker()
        chassis.header.stamp    = time_now
        chassis.header.frame_id = 'map'
        chassis.ns     = ns
        chassis.id     = id_offset + 0
        chassis.type   = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.scale.x = logic.body_length
        chassis.scale.y = logic.body_width
        chassis.scale.z = logic.body_height
        chassis.color.r = 0.0
        chassis.color.g = 0.5
        chassis.color.b = 0.0
        chassis.color.a = 1.0
        chassis.pose.position.x = body_x
        chassis.pose.position.y = body_y
        chassis.pose.position.z = body_z
        chassis.pose.orientation = q_body
        markers.append(chassis)

        # Wheels — local offset in body frame, rotated into world frame
        def add_wheel(marker_id, x_offset, y_side_multiplier):
            local_x = x_offset
            local_y = (logic.body_width / 2.0 + logic.wheel_width / 2.0) * y_side_multiplier
            local_z = -0.05
            wx, wy, wz = rotate_vec_by_quat(local_x, local_y, local_z, q_body)

            wheel = Marker()
            wheel.header.stamp    = time_now
            wheel.header.frame_id = 'map'
            wheel.ns     = ns
            wheel.id     = marker_id
            wheel.type   = Marker.CYLINDER
            wheel.action = Marker.ADD
            wheel.scale.x = logic.wheel_radius * 2
            wheel.scale.y = logic.wheel_radius * 2
            wheel.scale.z = logic.wheel_width
            wheel.color.r = 0.1
            wheel.color.g = 0.1
            wheel.color.b = 0.1
            wheel.color.a = 1.0
            wheel.pose.position.x = body_x + wx
            wheel.pose.position.y = body_y + wy
            wheel.pose.position.z = body_z + wz
            # wheel cylinder axis needs to lie along body-local Y, so compose
            # the body rotation with the original local "lay on its side" rotation
            q_wheel_local = euler_to_quaternion(yaw=0, pitch=1.5708, roll=0)
            wheel.pose.orientation = compose_quaternion(q_body, q_wheel_local)
            markers.append(wheel)

        x_positions = [-0.3, 0.0, 0.3]
        wheel_id = id_offset + 1
        for x_pos in x_positions:
            add_wheel(wheel_id, x_pos, 1)
            wheel_id += 1
            add_wheel(wheel_id, x_pos, -1)
            wheel_id += 1

        # Turret base
        turret_base = Marker()
        turret_base.header.stamp    = time_now
        turret_base.header.frame_id = 'map'
        turret_base.ns     = ns
        turret_base.id     = id_offset + 100
        turret_base.type   = Marker.CYLINDER
        turret_base.action = Marker.ADD
        turret_base.scale.x = logic.turret_radius * 2
        turret_base.scale.y = logic.turret_radius * 2
        turret_base.scale.z = logic.turret_height
        turret_base.color.r = 0.2
        turret_base.color.g = 0.4
        turret_base.color.b = 0.2
        turret_base.color.a = 1.0
        turret_base.pose.position.x = body_x
        turret_base.pose.position.y = body_y
        turret_base.pose.position.z = turret_world_z + logic.turret_height / 2.0
        turret_base.pose.orientation = q_turret
        markers.append(turret_base)

        # Cannon — local offset/orientation in turret frame, rotated into world
        cannon_local_x = (logic.cannon_length / 2.0) * math.cos(cannon_pitch)
        cannon_local_z = logic.turret_height + (logic.cannon_length / 2.0) * math.sin(cannon_pitch)
        cx, cy, cz = rotate_vec_by_quat(cannon_local_x, 0.0, cannon_local_z, q_turret)

        cannon = Marker()
        cannon.header.stamp    = time_now
        cannon.header.frame_id = 'map'
        cannon.ns     = ns
        cannon.id     = id_offset + 101
        cannon.type   = Marker.CYLINDER
        cannon.action = Marker.ADD
        cannon.scale.x = logic.cannon_radius * 2
        cannon.scale.y = logic.cannon_radius * 2
        cannon.scale.z = logic.cannon_length
        cannon.color.r = 0.1
        cannon.color.g = 0.1
        cannon.color.b = 0.1
        cannon.color.a = 1.0
        cannon.pose.position.x = body_x + cx
        cannon.pose.position.y = body_y + cy
        cannon.pose.position.z = turret_world_z + cz
        q_cannon_local = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + cannon_pitch, roll=0)
        cannon.pose.orientation = compose_quaternion(q_turret, q_cannon_local)
        markers.append(cannon)

        # Machine gun
        mg_mount_x = 0.05
        mg_mount_y = 0.15
        mg_mount_z_local = logic.turret_height + logic.mg_body_height / 2.0

        mbx, mby, mbz = rotate_vec_by_quat(mg_mount_x, mg_mount_y, mg_mount_z_local, q_turret)

        mg_body = Marker()
        mg_body.header.stamp    = time_now
        mg_body.header.frame_id = 'map'
        mg_body.ns     = ns
        mg_body.id     = id_offset + 200
        mg_body.type   = Marker.CUBE
        mg_body.action = Marker.ADD
        mg_body.scale.x = logic.mg_body_length
        mg_body.scale.y = logic.mg_body_width
        mg_body.scale.z = logic.mg_body_height
        mg_body.color.r = 0.25
        mg_body.color.g = 0.25
        mg_body.color.b = 0.25
        mg_body.color.a = 1.0
        mg_body.pose.position.x = body_x + mbx
        mg_body.pose.position.y = body_y + mby
        mg_body.pose.position.z = turret_world_z + mbz
        mg_body.pose.orientation = q_turret
        markers.append(mg_body)

        barrel_tip_offset_x = (logic.mg_barrel_length / 2.0) * math.cos(logic.mg_tilt)
        barrel_tip_offset_z = (logic.mg_barrel_length / 2.0) * math.sin(logic.mg_tilt)
        blx = mg_mount_x + barrel_tip_offset_x
        bly = mg_mount_y
        blz = mg_mount_z_local + barrel_tip_offset_z
        bbx, bby, bbz = rotate_vec_by_quat(blx, bly, blz, q_turret)

        mg_barrel = Marker()
        mg_barrel.header.stamp    = time_now
        mg_barrel.header.frame_id = 'map'
        mg_barrel.ns     = ns
        mg_barrel.id     = id_offset + 201
        mg_barrel.type   = Marker.CYLINDER
        mg_barrel.action = Marker.ADD
        mg_barrel.scale.x = logic.mg_barrel_radius * 2
        mg_barrel.scale.y = logic.mg_barrel_radius * 2
        mg_barrel.scale.z = logic.mg_barrel_length
        mg_barrel.color.r = 0.15
        mg_barrel.color.g = 0.15
        mg_barrel.color.b = 0.15
        mg_barrel.color.a = 1.0
        mg_barrel.pose.position.x = body_x + bbx
        mg_barrel.pose.position.y = body_y + bby
        mg_barrel.pose.position.z = turret_world_z + bbz
        q_barrel_local = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + logic.mg_tilt, roll=0)
        mg_barrel.pose.orientation = compose_quaternion(q_turret, q_barrel_local)
        markers.append(mg_barrel)

        amx = mg_mount_x
        amy = mg_mount_y + logic.mg_body_width / 2.0 + 0.04
        amz = mg_mount_z_local - 0.01
        abx, aby, abz = rotate_vec_by_quat(amx, amy, amz, q_turret)

        mg_ammo = Marker()
        mg_ammo.header.stamp    = time_now
        mg_ammo.header.frame_id = 'map'
        mg_ammo.ns     = ns
        mg_ammo.id     = id_offset + 202
        mg_ammo.type   = Marker.CUBE
        mg_ammo.action = Marker.ADD
        mg_ammo.scale.x = 0.10
        mg_ammo.scale.y = 0.06
        mg_ammo.scale.z = 0.06
        mg_ammo.color.r = 0.45
        mg_ammo.color.g = 0.35
        mg_ammo.color.b = 0.1
        mg_ammo.color.a = 1.0
        mg_ammo.pose.position.x = body_x + abx
        mg_ammo.pose.position.y = body_y + aby
        mg_ammo.pose.position.z = turret_world_z + abz
        mg_ammo.pose.orientation = q_turret
        markers.append(mg_ammo)

        return markers

    # ------------------------------------------------------------------
    # Main timer callback (20 Hz)
    # ------------------------------------------------------------------

    def timer_callback(self):
        time_now = self.get_clock().now().to_msg()
        dt = 0.05
        logic = self.logic

        # --- Advance game logic ---
        logic.update(dt)

        # --- Drain any log messages from the logic layer ---
        for msg in logic.pending_log_messages:
            self.get_logger().info(msg)
        logic.pending_log_messages.clear()

        # --- Build and publish full marker array ---
        marker_array = MarkerArray()

        # For the first couple of seconds after startup, broadcast a DELETEALL
        # marker so RViz clears out any stale/ghost markers left behind from a
        # previous run of this node (e.g. an old third tank, or an extra marker
        # like a stray "pipe", that no longer exists in the current code).
        if self._startup_clear_ticks > 0:
            clear_marker = Marker()
            clear_marker.header.stamp    = time_now
            clear_marker.header.frame_id = 'map'
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            self._startup_clear_ticks -= 1

        # Tank markers now get the current world pose baked in directly,
        # instead of being drawn relative to a TF frame at render time.
        for m in self.build_tank_markers(time_now, logic.x, logic.y, logic.theta,
                                          logic.turret_yaw, logic.cannon_pitch, id_offset=0):
            marker_array.markers.append(m)

        for m in self.build_tank_markers(time_now, logic.x2, logic.y2, logic.theta2,
                                          logic.turret_yaw2, logic.cannon_pitch2, id_offset=300):
            marker_array.markers.append(m)

        for m in self.build_projectile_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_mg_projectile_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_spark_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_wall_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_health_bar_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_game_over_marker(time_now):
            marker_array.markers.append(m)

        self.marker_array_pub.publish(marker_array)

        # --- Publish custom TankState messages ---
        self._publish_tank_states(time_now)


    # ------------------------------------------------------------------
    # Custom TankState publisher
    # ------------------------------------------------------------------

    def _publish_tank_states(self, time_now):
        """Build and publish a single TankState message for tank1."""
        logic = self.logic

        state = TankState()
        state.header.stamp    = time_now
        state.header.frame_id = 'map'
        state.x               = logic.x
        state.y               = logic.y
        state.theta           = logic.theta
        state.turret_yaw      = logic.turret_yaw
        state.cannon_pitch    = logic.cannon_pitch
        state.linear_velocity = logic.linear_velocity
        state.angular_velocity = logic.angular_velocity
        state.health          = logic.hp1
        state.max_health      = float(logic.MAX_HP)
        state.is_alive        = logic.hp1 > 0
        state.tank_name       = 'tank1'
        self.tank_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = TankBodyVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()