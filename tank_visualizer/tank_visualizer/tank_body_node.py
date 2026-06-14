import rclpy
from rclpy.node import Node
import math
import random

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from std_msgs.msg import Float32, Empty
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

        self.marker_array_pub = self.create_publisher(MarkerArray, '/tank_body_markers', 10)
        self.tf_broadcaster   = tf2_ros.TransformBroadcaster(self)

        # --- Tank 1 state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.turret_yaw    = 0.0
        self.cannon_pitch  = 0.0
        self.linear_velocity  = 0.0
        self.angular_velocity = 0.0

        # --- Tank 2 state (static, offset to the right) ---
        self.x2 = 0.0
        self.y2 = 3.0
        self.theta2       = 0.0
        self.turret_yaw2  = 0.0
        self.cannon_pitch2 = 0.0

        # -------------------------------------------------------
        # Projectile + spark state
        # Marker ID ranges (must not overlap tanks):
        #   Tank 1:        0 – 299
        #   Tank 2:      300 – 599
        #   Cannon shells: 600 – 699   (max 100 in flight)
        #   MG bullets:    700 – 799   (max 100 in flight)
        #   Sparks:       800 – 2099   (max 1300 active particles)
        # -------------------------------------------------------
        self.projectiles      = []        # cannon shells
        self.mg_projectiles   = []        # MG bullets
        self.sparks           = []
        self.next_proj_id     = 600
        self.next_mg_proj_id  = 700
        self.next_spark_id    = 800
        self.prev_proj_ids    = set()
        self.prev_mg_proj_ids = set()
        self.prev_spark_ids   = set()
        self.fire_cooldown    = 0.0       # cannon cooldown
        self.mg_cooldown      = 0.0       # MG cooldown

        self.create_subscription(Float32, '/turret/yaw_command',   self.turret_yaw_callback,   10)
        self.create_subscription(Float32, '/turret/pitch_command', self.cannon_pitch_callback,  10)
        self.create_subscription(Twist,   '/cmd_vel',              self.cmd_vel_callback,       10)
        self.create_subscription(Empty,   '/tank/fire',            self.fire_callback,          10)
        self.create_subscription(Empty,   '/tank/fire_mg',         self.fire_mg_callback,       10)

        # Tank dimensions
        self.body_length   = 1.0
        self.body_width    = 0.6
        self.body_height   = 0.3
        self.wheel_radius  = 0.15
        self.wheel_width   = 0.1
        self.turret_radius = 0.25
        self.turret_height = 0.2
        self.cannon_length = 0.6
        self.cannon_radius = 0.05

        self.mg_body_length   = 0.18
        self.mg_body_width    = 0.08
        self.mg_body_height   = 0.08
        self.mg_barrel_length = 0.28
        self.mg_barrel_radius = 0.018
        self.mg_tilt          = 0.15

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Tank Body Visualizer Node Started')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def turret_yaw_callback(self, msg):
        self.turret_yaw = msg.data

    def cannon_pitch_callback(self, msg):
        self.cannon_pitch = max(-0.785, min(0.785, msg.data))

    def cmd_vel_callback(self, msg):
        self.linear_velocity  = msg.linear.x
        self.angular_velocity = msg.angular.z

    # ------------------------------------------------------------------
    # FIRE — spawn projectile at cannon tip in world-space direction
    # ------------------------------------------------------------------

    def fire_callback(self, msg):
        if self.fire_cooldown > 0.0:
            return  # still cooling down

        # Total world-space yaw the cannon is pointing
        total_yaw = self.theta + self.turret_yaw
        pitch     = self.cannon_pitch

        # Turret frame origin in world Z
        #   tank body is at z=0.15 in map (see TF below)
        #   turret frame is body_height/2 above that
        turret_origin_z = 0.15 + self.body_height / 2.0   # = 0.30 m

        # Cannon tip offset from turret origin (in turret-local X and Z)
        cannon_horiz = self.cannon_length * math.cos(pitch)   # forward reach
        cannon_vert  = self.turret_height + self.cannon_length * math.sin(pitch)

        # Rotate horizontal reach by world yaw to get world-space tip
        tip_x = self.x + cannon_horiz * math.cos(total_yaw)
        tip_y = self.y + cannon_horiz * math.sin(total_yaw)
        tip_z = turret_origin_z + cannon_vert

        # Projectile velocity (world frame)
        speed = 8.0   # m/s  — adjust to taste
        vx = speed * math.cos(pitch) * math.cos(total_yaw)
        vy = speed * math.cos(pitch) * math.sin(total_yaw)
        vz = speed * math.sin(pitch)

        # Assign a recycling ID (600-699)
        proj_id          = self.next_proj_id
        self.next_proj_id = 600 if self.next_proj_id >= 699 else self.next_proj_id + 1

        self.projectiles.append({
            'id':       proj_id,
            'type':     'cannon',
            'x': tip_x, 'y': tip_y, 'z': tip_z,
            'vx': vx,   'vy': vy,   'vz': vz,
            'lifetime': 4.0,        # auto-expire after 4 s
        })

        # Brief muzzle flash at cannon tip
        self._create_spark(tip_x, tip_y, tip_z, lifetime=0.15, is_muzzle=True)

        self.fire_cooldown = 0.4    # 0.4 s between cannon shots

    # ------------------------------------------------------------------
    # MG FIRE — spawn small bullet from MG barrel tip
    # ------------------------------------------------------------------

    def fire_mg_callback(self, msg):
        if self.mg_cooldown > 0.0:
            return  # still cooling down

        total_yaw = self.theta + self.turret_yaw

        # MG barrel mounts at (mg_mount_x, mg_mount_y) in turret frame,
        # tilted upward by mg_tilt, barrel extends mg_barrel_length forward
        mg_mount_x = 0.05
        mg_mount_y = 0.15
        mg_mount_z = 0.15 + self.body_height / 2.0 + self.turret_height + self.mg_body_height / 2.0

        # Full barrel tip offset in turret-local frame
        barrel_reach_x = self.mg_barrel_length * math.cos(self.mg_tilt)
        barrel_reach_z = self.mg_barrel_length * math.sin(self.mg_tilt)

        # Rotate turret-local XY offset into world frame
        local_x = mg_mount_x + barrel_reach_x
        local_y = mg_mount_y

        tip_x = self.x + local_x * math.cos(total_yaw) - local_y * math.sin(total_yaw)
        tip_y = self.y + local_x * math.sin(total_yaw) + local_y * math.cos(total_yaw)
        tip_z = mg_mount_z + barrel_reach_z

        # MG fires flat along turret yaw; gravity handles the drop
        speed = 14.0   # faster than cannon shell
        vx = speed * math.cos(total_yaw)
        vy = speed * math.sin(total_yaw)
        vz = 0.0

        mg_id              = self.next_mg_proj_id
        self.next_mg_proj_id = 700 if self.next_mg_proj_id >= 799 else self.next_mg_proj_id + 1

        self.mg_projectiles.append({
            'id':       mg_id,
            'x': tip_x, 'y': tip_y, 'z': tip_z,
            'vx': vx,   'vy': vy,   'vz': vz,
            'lifetime': 2.0,
        })

        # Tiny muzzle flash
        self._create_spark(tip_x, tip_y, tip_z, lifetime=0.08, is_muzzle=True)

        self.mg_cooldown = 0.1    # 10 rounds/sec

    # ------------------------------------------------------------------
    # Spark factory
    # ------------------------------------------------------------------

    def _create_spark(self, x, y, z, lifetime=0.5, is_muzzle=False):
        """
        Spawn a burst of spark particles at world position (x, y, z).

        Each particle gets its own random velocity so they fan out like
        real sparks flying off an impact or muzzle flash.

        is_muzzle=True  → compact white-hot flash (fewer, faster, brighter)
        is_muzzle=False → impact burst (more particles, orange/red, drift wider)
        """
        num_particles = 5 if is_muzzle else 14
        base_speed    = 3.0 if is_muzzle else 2.0

        for _ in range(num_particles):
            spark_id           = self.next_spark_id
            self.next_spark_id = 800 if self.next_spark_id >= 2099 else self.next_spark_id + 1

            # Random direction spread (full sphere for impact, forward cone for muzzle)
            if is_muzzle:
                # Tight forward cone ± 60°
                yaw_spread   = random.uniform(-math.pi / 3, math.pi / 3)
                pitch_spread = random.uniform(-math.pi / 4, math.pi / 4)
            else:
                # Full hemisphere upward (sparks bounce off surface)
                yaw_spread   = random.uniform(-math.pi, math.pi)
                pitch_spread = random.uniform(0.0, math.pi / 2)

            speed = base_speed * random.uniform(0.4, 1.0)
            vx = speed * math.cos(pitch_spread) * math.cos(yaw_spread)
            vy = speed * math.cos(pitch_spread) * math.sin(yaw_spread)
            vz = speed * math.sin(pitch_spread) + (0.5 if not is_muzzle else 0.0)

            part_lifetime = lifetime * random.uniform(0.6, 1.0)

            self.sparks.append({
                'id':             spark_id,
                'x': x, 'y': y, 'z': z,
                'vx': vx, 'vy': vy, 'vz': vz,
                'lifetime':       part_lifetime,
                'total_lifetime': part_lifetime,
                'is_muzzle':      is_muzzle,
            })

    # ------------------------------------------------------------------
    # Per-tick updates
    # ------------------------------------------------------------------

    def update_projectiles(self, dt):
        """Move projectiles, check collisions, spawn sparks on hit."""
        still_active = []

        # Tank 2 bounding-sphere centre and radius
        t2cx = self.x2
        t2cy = self.y2
        t2cz = 0.15 + self.body_height / 2.0   # same z logic as tank TF
        hit_radius = 0.55                         # generous hitbox

        for p in self.projectiles:
            # Integrate position
            p['x'] += p['vx'] * dt
            p['y'] += p['vy'] * dt
            p['z'] += p['vz'] * dt
            p['lifetime'] -= dt

            # Discard if expired (no spark — it just vanished off-screen)
            if p['lifetime'] <= 0:
                continue

            hit = False

            # --- Ground hit ---
            if p['z'] < 0.1:
                self._create_spark(p['x'], p['y'], 0.1, lifetime=0.5, is_muzzle=False)
                hit = True

            # --- Tank 2 hit ---
            if not hit:
                d = math.sqrt(
                    (p['x'] - t2cx) ** 2 +
                    (p['y'] - t2cy) ** 2 +
                    (p['z'] - t2cz) ** 2
                )
                if d < hit_radius:
                    self._create_spark(p['x'], p['y'], p['z'], lifetime=0.6, is_muzzle=False)
                    hit = True

            if not hit:
                still_active.append(p)

        self.projectiles = still_active

    def update_mg_projectiles(self, dt):
        """Move MG bullets, check collisions, spawn small sparks on hit."""
        GRAVITY = -9.81   # m/s² — real gravity so bullets drop to target height
        still_active = []

        t2cx = self.x2
        t2cy = self.y2
        t2cz = 0.15 + self.body_height / 2.0
        hit_radius = 0.55

        for p in self.mg_projectiles:
            p['vz'] += GRAVITY * dt   # gravity pulls bullet down each tick
            p['x'] += p['vx'] * dt
            p['y'] += p['vy'] * dt
            p['z'] += p['vz'] * dt
            p['lifetime'] -= dt

            if p['lifetime'] <= 0:
                continue

            hit = False

            # Ground hit — tiny spark
            if p['z'] < 0.1:
                self._create_spark(p['x'], p['y'], 0.1, lifetime=0.2, is_muzzle=False)
                hit = True

            # Tank 2 hit — small spark
            if not hit:
                d = math.sqrt(
                    (p['x'] - t2cx) ** 2 +
                    (p['y'] - t2cy) ** 2 +
                    (p['z'] - t2cz) ** 2
                )
                if d < hit_radius:
                    self._create_spark(p['x'], p['y'], p['z'], lifetime=0.25, is_muzzle=False)
                    hit = True

            if not hit:
                still_active.append(p)

        self.mg_projectiles = still_active

    def update_sparks(self, dt):
        """Tick down spark lifetimes, move each particle, apply gravity, remove dead ones."""
        GRAVITY = -4.0   # m/s²  — lighter than real so sparks arc nicely
        new_sparks = []
        for s in self.sparks:
            s['lifetime'] -= dt
            if s['lifetime'] <= 0:
                continue
            # Integrate position
            s['x']  += s['vx'] * dt
            s['y']  += s['vy'] * dt
            s['z']  += s['vz'] * dt
            # Gravity drags Z velocity down
            s['vz'] += GRAVITY * dt
            # Stop at ground
            if s['z'] < 0.02:
                s['z']  = 0.02
                s['vz'] = 0.0
            new_sparks.append(s)
        self.sparks = new_sparks

    # ------------------------------------------------------------------
    # Marker builders — projectiles + sparks
    # ------------------------------------------------------------------

    def build_projectile_markers(self, time_now):
        markers   = []
        active_ids = set()

        for p in self.projectiles:
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

        for old_id in self.prev_proj_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'projectiles'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        self.prev_proj_ids = active_ids
        return markers

    def build_mg_projectile_markers(self, time_now):
        """Small silver/grey bullets from the machine gun."""
        markers    = []
        active_ids = set()

        for p in self.mg_projectiles:
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

        for old_id in self.prev_mg_proj_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'mg_projectiles'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        self.prev_mg_proj_ids = active_ids
        return markers

    def build_spark_markers(self, time_now):
        """
        Each particle ages from white-hot → orange → red → transparent.

        Muzzle particles: smaller, blue-white hot core.
        Impact particles: larger burst, orange-yellow arc then fade red.

        frac = remaining_lifetime / total_lifetime  (1.0 → 0.0)
        """
        markers    = []
        active_ids = set()

        for s in self.sparks:
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
        for old_id in self.prev_spark_ids - active_ids:
            d = Marker()
            d.header.stamp    = time_now
            d.header.frame_id = 'map'
            d.ns     = 'sparks'
            d.id     = old_id
            d.action = Marker.DELETE
            markers.append(d)

        self.prev_spark_ids = active_ids
        return markers

    # ------------------------------------------------------------------
    # Tank marker builder (unchanged from original)
    # ------------------------------------------------------------------

    def build_tank_markers(self, time_now, frame_body, frame_turret,
                           turret_yaw, cannon_pitch, id_offset):
        """Build all markers for one tank. id_offset separates tank1 (0) from tank2 (300)."""
        markers = []

        # Chassis
        chassis = Marker()
        chassis.header.stamp    = time_now
        chassis.header.frame_id = frame_body
        chassis.id     = id_offset + 0
        chassis.type   = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.scale.x = self.body_length
        chassis.scale.y = self.body_width
        chassis.scale.z = self.body_height
        chassis.color.r = 0.0
        chassis.color.g = 0.5
        chassis.color.b = 0.0
        chassis.color.a = 1.0
        markers.append(chassis)

        # Wheels
        def add_wheel(marker_id, x_offset, y_side_multiplier):
            wheel = Marker()
            wheel.header.stamp    = time_now
            wheel.header.frame_id = frame_body
            wheel.id     = marker_id
            wheel.type   = Marker.CYLINDER
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
        turret_base.header.frame_id = frame_turret
        turret_base.id     = id_offset + 100
        turret_base.type   = Marker.CYLINDER
        turret_base.action = Marker.ADD
        turret_base.scale.x = self.turret_radius * 2
        turret_base.scale.y = self.turret_radius * 2
        turret_base.scale.z = self.turret_height
        turret_base.color.r = 0.2
        turret_base.color.g = 0.4
        turret_base.color.b = 0.2
        turret_base.color.a = 1.0
        turret_base.pose.position.z = self.turret_height / 2.0
        markers.append(turret_base)

        # Cannon
        cannon = Marker()
        cannon.header.stamp    = time_now
        cannon.header.frame_id = frame_turret
        cannon.id     = id_offset + 101
        cannon.type   = Marker.CYLINDER
        cannon.action = Marker.ADD
        cannon.scale.x = self.cannon_radius * 2
        cannon.scale.y = self.cannon_radius * 2
        cannon.scale.z = self.cannon_length
        cannon.color.r = 0.1
        cannon.color.g = 0.1
        cannon.color.b = 0.1
        cannon.color.a = 1.0
        cannon.pose.position.x = (self.cannon_length / 2.0) * math.cos(cannon_pitch)
        cannon.pose.position.z = self.turret_height + (self.cannon_length / 2.0) * math.sin(cannon_pitch)
        cannon.pose.orientation = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + cannon_pitch, roll=0)
        markers.append(cannon)

        # Machine gun
        mg_mount_x = 0.05
        mg_mount_y = 0.15
        mg_mount_z = self.turret_height + self.mg_body_height / 2.0

        mg_body = Marker()
        mg_body.header.stamp    = time_now
        mg_body.header.frame_id = frame_turret
        mg_body.id     = id_offset + 200
        mg_body.type   = Marker.CUBE
        mg_body.action = Marker.ADD
        mg_body.scale.x = self.mg_body_length
        mg_body.scale.y = self.mg_body_width
        mg_body.scale.z = self.mg_body_height
        mg_body.color.r = 0.25
        mg_body.color.g = 0.25
        mg_body.color.b = 0.25
        mg_body.color.a = 1.0
        mg_body.pose.position.x = mg_mount_x
        mg_body.pose.position.y = mg_mount_y
        mg_body.pose.position.z = mg_mount_z
        mg_body.pose.orientation = euler_to_quaternion(yaw=0, pitch=0, roll=0)
        markers.append(mg_body)

        barrel_tip_offset_x = (self.mg_barrel_length / 2.0) * math.cos(self.mg_tilt)
        barrel_tip_offset_z = (self.mg_barrel_length / 2.0) * math.sin(self.mg_tilt)

        mg_barrel = Marker()
        mg_barrel.header.stamp    = time_now
        mg_barrel.header.frame_id = frame_turret
        mg_barrel.id     = id_offset + 201
        mg_barrel.type   = Marker.CYLINDER
        mg_barrel.action = Marker.ADD
        mg_barrel.scale.x = self.mg_barrel_radius * 2
        mg_barrel.scale.y = self.mg_barrel_radius * 2
        mg_barrel.scale.z = self.mg_barrel_length
        mg_barrel.color.r = 0.15
        mg_barrel.color.g = 0.15
        mg_barrel.color.b = 0.15
        mg_barrel.color.a = 1.0
        mg_barrel.pose.position.x = mg_mount_x + barrel_tip_offset_x
        mg_barrel.pose.position.y = mg_mount_y
        mg_barrel.pose.position.z = mg_mount_z + barrel_tip_offset_z
        mg_barrel.pose.orientation = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + self.mg_tilt, roll=0)
        markers.append(mg_barrel)

        mg_ammo = Marker()
        mg_ammo.header.stamp    = time_now
        mg_ammo.header.frame_id = frame_turret
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
        mg_ammo.pose.position.x = mg_mount_x
        mg_ammo.pose.position.y = mg_mount_y + self.mg_body_width / 2.0 + 0.04
        mg_ammo.pose.position.z = mg_mount_z - 0.01
        mg_ammo.pose.orientation = euler_to_quaternion(yaw=0, pitch=0, roll=0)
        markers.append(mg_ammo)

        return markers

    # ------------------------------------------------------------------
    # Main timer callback (20 Hz)
    # ------------------------------------------------------------------

    def timer_callback(self):
        time_now = self.get_clock().now().to_msg()
        dt = 0.05

        # --- Tank 1 movement ---
        self.theta += self.angular_velocity * dt
        self.x    += self.linear_velocity * math.cos(self.theta) * dt
        self.y    += self.linear_velocity * math.sin(self.theta) * dt

        # --- Fire cooldown tick ---
        self.fire_cooldown = max(0.0, self.fire_cooldown - dt)
        self.mg_cooldown   = max(0.0, self.mg_cooldown   - dt)

        # --- Update projectiles and sparks ---
        self.update_projectiles(dt)
        self.update_mg_projectiles(dt)
        self.update_sparks(dt)

        # --- Tank 1 TF ---
        t = TransformStamped()
        t.header.stamp      = time_now
        t.header.frame_id   = 'map'
        t.child_frame_id    = 'tank_body'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.15
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        t_turret = TransformStamped()
        t_turret.header.stamp    = time_now
        t_turret.header.frame_id = 'tank_body'
        t_turret.child_frame_id  = 'tank_turret'
        t_turret.transform.translation.z = self.body_height / 2.0
        t_turret.transform.rotation.z = math.sin(self.turret_yaw / 2.0)
        t_turret.transform.rotation.w = math.cos(self.turret_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t_turret)

        # --- Tank 2 TF (static) ---
        t2 = TransformStamped()
        t2.header.stamp    = time_now
        t2.header.frame_id = 'map'
        t2.child_frame_id  = 'tank_body_2'
        t2.transform.translation.x = self.x2
        t2.transform.translation.y = self.y2
        t2.transform.translation.z = 0.15
        t2.transform.rotation.z = math.sin(self.theta2 / 2.0)
        t2.transform.rotation.w = math.cos(self.theta2 / 2.0)
        self.tf_broadcaster.sendTransform(t2)

        t_turret2 = TransformStamped()
        t_turret2.header.stamp    = time_now
        t_turret2.header.frame_id = 'tank_body_2'
        t_turret2.child_frame_id  = 'tank_turret_2'
        t_turret2.transform.translation.z = self.body_height / 2.0
        t_turret2.transform.rotation.z = math.sin(self.turret_yaw2 / 2.0)
        t_turret2.transform.rotation.w = math.cos(self.turret_yaw2 / 2.0)
        self.tf_broadcaster.sendTransform(t_turret2)

        # --- Build and publish full marker array ---
        marker_array = MarkerArray()

        for m in self.build_tank_markers(time_now, 'tank_body', 'tank_turret',
                                          self.turret_yaw, self.cannon_pitch, id_offset=0):
            marker_array.markers.append(m)

        for m in self.build_tank_markers(time_now, 'tank_body_2', 'tank_turret_2',
                                          self.turret_yaw2, self.cannon_pitch2, id_offset=300):
            marker_array.markers.append(m)

        for m in self.build_projectile_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_mg_projectile_markers(time_now):
            marker_array.markers.append(m)

        for m in self.build_spark_markers(time_now):
            marker_array.markers.append(m)

        self.marker_array_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TankBodyVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()