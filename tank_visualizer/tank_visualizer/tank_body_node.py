import rclpy
from rclpy.node import Node
import math
import random

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32, Empty

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
    def __init__(self):
        super().__init__('tank_body_visualizer')

        self.marker_array_pub = self.create_publisher(MarkerArray, '/tank_body_markers', 10)

        # --- Tank 1 state (player, keyboard controlled) ---
        # Spawn point moved off-center on the open grid (previously 0,0).
        # Tweak these two values if you want to nudge the exact spot further.
        self.x = -3.0
        self.y =  1.5
        self.theta = 0.0
        self.turret_yaw    = 0.0
        self.cannon_pitch  = 0.0
        self.linear_velocity  = 0.0
        self.angular_velocity = 0.0

        # --- Tank 2 state (autonomous tank) ---
        self.turret_yaw2   = 0.0
        self.cannon_pitch2 = 0.0

        # will be positioned properly after wall segments are built (see below)
        self.x2 = 0.0
        self.y2 = 0.0
        self.theta2 = 0.0

        self.auto_speed         = 0.4   # slow enough to give player a chance
        self.auto_turn_speed    = 1.0   # rad/s max turn rate
        self.auto_target_x      = 0.0
        self.auto_target_y      = 0.0
        self.auto_waypoints     = []    # filled in after wall segments are built
        self.auto_wp_index      = 0
        self.auto_fire_cooldown = random.uniform(2.0, 3.0)
        self.auto_mg_cooldown   = random.uniform(1.2, 2.0)
        self.startup_grace_period = 5.0

        # --- Health / combat state ---
        self.MAX_HP        = 20.0
        self.hp1            = float(self.MAX_HP)   # player tank
        self.hp2            = float(self.MAX_HP)   # auto tank
        self.CANNON_DAMAGE  = 5.0
        self.MG_DAMAGE      = 5.0
        self.game_over       = False
        self.winner          = None   # 'tank1' or 'tank2'

        # -------------------------------------------------------
        # Projectile + spark state
        # Marker ID ranges (must not overlap tanks):
        #   Tank 1:        0 – 299
        #   Tank 2:      300 – 599
        #   Cannon shells: 600 – 699   (max 100 in flight)
        #   MG bullets:    700 – 799   (max 100 in flight)
        #   Sparks:       800 – 2099   (max 1300 active particles)
        #   Walls:        3000+
        #   Health bars:  5000 – 5019
        #   Game over:    6000
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

        # Number of timer ticks (at 20 Hz) during which a DELETEALL marker
        # is broadcast on startup. This force-clears any stale/ghost markers
        # left over in RViz from a previous version of this node (e.g. an
        # old third tank or extra marker that no longer exists in the code
        # but never received an explicit DELETE), without requiring the
        # user to manually hit "Reset" in RViz.
        self._startup_clear_ticks = 40   # ~2 seconds at 20 Hz

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

        # --- Wall / corridor dimensions ---
        self.wall_thickness  = 0.1
        self.wall_height     = 0.4
        self.corridor_width  = 1.8     # clear space between inner wall faces (tank body_width = 0.6)
        self.segment_length  = 6.0     # length of each straight section
        self.corner_origin   = (1.5, -2.0)  # where the corridor starts, tweak to taste

        self.wall_segments = self._build_wall_segments()
        self.wall_aabbs    = self._compute_wall_aabbs()

        # Bounding circle radius used for tank-vs-wall collision checks.
        # (Half the tank's diagonal — a safe over-approximation of the rectangular body.)
        self.collision_radius = math.hypot(self.body_length / 2.0, self.body_width / 2.0)

        # --- Spawn Tank 2 at the entrance to the corridor ---
        ox, oy = self.corner_origin
        L  = self.segment_length
        hw = self.corridor_width / 2.0
        self.x2 = ox + L
        self.y2 = oy
        self.theta2 = math.pi   # facing left along the corridor

        # Path for Tank 2 to follow: along the corridor, then toward the player.
        self.auto_waypoints = [
            (ox + hw, oy),     # move left along the corridor toward the corner
            (ox, oy),          # corridor entrance
            (0.0, 0.0),        # into the open grid, near center
        ]
        self.auto_wp_index = 0

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
        if self.game_over:
            return
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
            'source':   'tank1',
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
        if self.game_over:
            return
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
            'source':   'tank1',
            'x': tip_x, 'y': tip_y, 'z': tip_z,
            'vx': vx,   'vy': vy,   'vz': vz,
            'lifetime': 2.0,
        })

        # Tiny muzzle flash
        self._create_spark(tip_x, tip_y, tip_z, lifetime=0.08, is_muzzle=True)

        self.mg_cooldown = 0.1    # 10 rounds/sec

    # ------------------------------------------------------------------
    # Tank 2 — autonomous path following + random firing
    # ------------------------------------------------------------------

    def update_tank2_auto(self, dt):
        if self.game_over:
            return

        if self.auto_wp_index >= len(self.auto_waypoints):
            # Completed the path; hold position and aim at the player.
            desired_turret = math.atan2(self.y - self.y2, self.x - self.x2) - self.theta2
            desired_turret = (desired_turret + math.pi) % (2 * math.pi) - math.pi
            self.turret_yaw2 += max(-0.1 * dt, min(0.1 * dt, desired_turret))

            self.auto_fire_cooldown -= dt
            if self.auto_fire_cooldown <= 0.0:
                self._tank2_fire_cannon()
                self.auto_fire_cooldown = random.uniform(2.5, 4.0)

            self.auto_mg_cooldown -= dt
            if self.auto_mg_cooldown <= 0.0:
                self._tank2_fire_mg()
                self.auto_mg_cooldown = random.uniform(1.5, 2.5)
            return

        target_x, target_y = self.auto_waypoints[self.auto_wp_index]
        dx = target_x - self.x2
        dy = target_y - self.y2
        dist = math.hypot(dx, dy)

        if dist < 0.25:
            self.auto_wp_index += 1
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_diff = (desired_yaw - self.theta2 + math.pi) % (2 * math.pi) - math.pi
        max_turn = self.auto_turn_speed * dt
        self.theta2 += max(-max_turn, min(max_turn, yaw_diff))

        if abs(yaw_diff) < 0.35:
            candidate_x = self.x2 + self.auto_speed * math.cos(self.theta2) * dt
            candidate_y = self.y2 + self.auto_speed * math.sin(self.theta2) * dt
            if not self._collides_with_walls(candidate_x, candidate_y):
                self.x2 = candidate_x
                self.y2 = candidate_y
            elif not self._collides_with_walls(candidate_x, self.y2):
                self.x2 = candidate_x
            elif not self._collides_with_walls(self.x2, candidate_y):
                self.y2 = candidate_y

        self.turret_yaw2 += random.uniform(-0.1, 0.1) * dt

        self.auto_fire_cooldown -= dt
        if self.auto_fire_cooldown <= 0.0:
            self._tank2_fire_cannon()
            self.auto_fire_cooldown = random.uniform(2.0, 3.5)

        self.auto_mg_cooldown -= dt
        if self.auto_mg_cooldown <= 0.0:
            self._tank2_fire_mg()
            self.auto_mg_cooldown = random.uniform(1.2, 2.5)

    def _tank2_fire_cannon(self):
        total_yaw = self.theta2 + self.turret_yaw2 + random.uniform(-0.6, 0.6)
        pitch = random.uniform(-0.2, 0.2)

        turret_origin_z = 0.15 + self.body_height / 2.0
        cannon_horiz = self.cannon_length * math.cos(pitch)
        cannon_vert  = self.turret_height + self.cannon_length * math.sin(pitch)

        tip_x = self.x2 + cannon_horiz * math.cos(total_yaw)
        tip_y = self.y2 + cannon_horiz * math.sin(total_yaw)
        tip_z = turret_origin_z + cannon_vert

        speed = 8.0
        vx = speed * math.cos(pitch) * math.cos(total_yaw)
        vy = speed * math.cos(pitch) * math.sin(total_yaw)
        vz = speed * math.sin(pitch)

        proj_id = self.next_proj_id
        self.next_proj_id = 600 if self.next_proj_id >= 699 else self.next_proj_id + 1

        self.projectiles.append({
            'id': proj_id, 'type': 'cannon', 'source': 'tank2',
            'x': tip_x, 'y': tip_y, 'z': tip_z,
            'vx': vx, 'vy': vy, 'vz': vz,
            'lifetime': 4.0,
        })
        self._create_spark(tip_x, tip_y, tip_z, lifetime=0.15, is_muzzle=True)

    def _tank2_fire_mg(self):
        total_yaw = self.theta2 + self.turret_yaw2 + random.uniform(-0.8, 0.8)
        mg_mount_z = 0.15 + self.body_height / 2.0 + self.turret_height + self.mg_body_height / 2.0

        tip_x = self.x2 + 0.2 * math.cos(total_yaw)
        tip_y = self.y2 + 0.2 * math.sin(total_yaw)
        tip_z = mg_mount_z

        speed = 14.0
        vx = speed * math.cos(total_yaw)
        vy = speed * math.sin(total_yaw)
        vz = 0.0

        mg_id = self.next_mg_proj_id
        self.next_mg_proj_id = 700 if self.next_mg_proj_id >= 799 else self.next_mg_proj_id + 1

        self.mg_projectiles.append({
            'id': mg_id, 'source': 'tank2',
            'x': tip_x, 'y': tip_y, 'z': tip_z,
            'vx': vx, 'vy': vy, 'vz': vz, 'lifetime': 2.0,
        })
        self._create_spark(tip_x, tip_y, tip_z, lifetime=0.08, is_muzzle=True)

    # ------------------------------------------------------------------
    # Damage / win-loss
    # ------------------------------------------------------------------

    def _apply_damage(self, shooter_source, dmg):
        """shooter_source is who FIRED the shot; damage lands on the other tank."""
        if self.game_over:
            return

        # Give the player a short spawn grace period so Tank 2 cannot
        # immediately end the match before the first movement input.
        if self.startup_grace_period > 0.0 and shooter_source == 'tank2':
            return

        if shooter_source == 'tank1':
            self.hp2 = max(0, self.hp2 - dmg)
            if self.hp2 <= 0:
                self.game_over = True
                self.winner = 'tank1'
                self.get_logger().info('Tank 2 destroyed — YOU WIN!')
        else:
            self.hp1 = max(0, self.hp1 - dmg)
            if self.hp1 <= 0:
                self.game_over = True
                self.winner = 'tank2'
                self.get_logger().info('Tank 1 destroyed — YOU LOSE!')

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
        """Move projectiles, check collisions, spawn sparks + apply damage on hit."""
        still_active = []

        t1cx, t1cy, t1cz = self.x,  self.y,  0.15 + self.body_height / 2.0
        t2cx, t2cy, t2cz = self.x2, self.y2, 0.15 + self.body_height / 2.0
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

            # --- Tank hit (whichever tank did NOT fire this shot) ---
            if not hit:
                source = p.get('source', 'tank1')
                if source == 'tank2':
                    tx, ty, tz = t1cx, t1cy, t1cz
                else:
                    tx, ty, tz = t2cx, t2cy, t2cz

                d = math.sqrt(
                    (p['x'] - tx) ** 2 +
                    (p['y'] - ty) ** 2 +
                    (p['z'] - tz) ** 2
                )
                if d < hit_radius:
                    self._create_spark(p['x'], p['y'], p['z'], lifetime=0.6, is_muzzle=False)
                    self._apply_damage(source, self.CANNON_DAMAGE)
                    hit = True

            if not hit:
                still_active.append(p)

        self.projectiles = still_active

    def update_mg_projectiles(self, dt):
        """Move MG bullets, check collisions, spawn small sparks + apply damage on hit."""
        GRAVITY = -9.81   # m/s² — real gravity so bullets drop to target height
        still_active = []

        t1cx, t1cy, t1cz = self.x,  self.y,  0.15 + self.body_height / 2.0
        t2cx, t2cy, t2cz = self.x2, self.y2, 0.15 + self.body_height / 2.0
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

            # Tank hit — small spark + damage
            if not hit:
                source = p.get('source', 'tank1')
                if source == 'tank2':
                    tx, ty, tz = t1cx, t1cy, t1cz
                else:
                    tx, ty, tz = t2cx, t2cy, t2cz

                d = math.sqrt(
                    (p['x'] - tx) ** 2 +
                    (p['y'] - ty) ** 2 +
                    (p['z'] - tz) ** 2
                )
                if d < hit_radius:
                    self._create_spark(p['x'], p['y'], p['z'], lifetime=0.25, is_muzzle=False)
                    self._apply_damage(source, self.MG_DAMAGE)
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
    # Health bar + game-over builders
    # ------------------------------------------------------------------
    def build_health_bar_markers(self, time_now):
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

        make_bar(self.x,  self.y,  self.theta,  self.hp1, self.MAX_HP, 5000)
        make_bar(self.x2, self.y2, self.theta2, self.hp2, self.MAX_HP, 5010)

        return markers

    def build_game_over_marker(self, time_now):
        if not self.game_over:
            return []

        text = Marker()
        text.header.stamp    = time_now
        text.header.frame_id = 'map'
        text.ns     = 'game_over'
        text.id     = 6000
        text.type   = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.8
        if self.winner == 'tank1':
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
    # Wall / corridor builders
    # ------------------------------------------------------------------

    def _build_wall_segments(self):
        """
        Build a right-angle corridor: straight east, turn right, straight south.
        Returns a list of (x_center, y_center, length, thickness, yaw) for CUBE markers.
        Outer wall corner is squared off, inner wall corner is cut back,
        so the turn is actually driveable.
        """
        ox, oy = self.corner_origin
        L  = self.segment_length
        hw = self.corridor_width / 2.0
        t  = self.wall_thickness

        segs = []

        # --- Outer wall (the wide/outside of the turn) ---
        # Segment 1: runs along x, from ox to ox+L+hw (extended to cover the corner)
        seg1_len = L + hw
        segs.append((ox + seg1_len / 2.0, oy + hw, seg1_len, t, 0.0))

        # Segment 2: runs along y (south), from oy+hw down to oy-L
        seg2_len = L + hw
        segs.append((ox + L + hw, oy + hw - seg2_len / 2.0, seg2_len, t, math.pi / 2.0))

        # --- Inner wall (the tight/inside of the turn) ---
        # Segment 1: cut short so the corner opening isn't blocked
        seg3_len = L - hw
        segs.append((ox + seg3_len / 2.0, oy - hw, seg3_len, t, 0.0))

        # Segment 2: also cut short
        seg4_len = L - hw
        segs.append((ox + L - hw, oy - hw - seg4_len / 2.0, seg4_len, t, math.pi / 2.0))

        return segs

    def _compute_wall_aabbs(self):
        """
        Convert each wall segment into an axis-aligned bounding box
        (xmin, xmax, ymin, ymax). Safe because every wall segment's yaw
        is either 0 (horizontal) or pi/2 (vertical) — no diagonal walls.
        """
        aabbs = []
        for (cx, cy, length, thickness, yaw) in self.wall_segments:
            half_len   = length / 2.0
            half_thick = thickness / 2.0
            if abs(math.sin(yaw)) < 0.5:
                # horizontal segment: long along x, thin along y
                aabbs.append((cx - half_len, cx + half_len, cy - half_thick, cy + half_thick))
            else:
                # vertical segment: thin along x, long along y
                aabbs.append((cx - half_thick, cx + half_thick, cy - half_len, cy + half_len))
        return aabbs

    def _collides_with_walls(self, x, y):
        """
        Circle-vs-AABB test: True if a circle of radius self.collision_radius
        centered at (x, y) overlaps any wall's bounding box.
        """
        r2 = self.collision_radius * self.collision_radius
        for (xmin, xmax, ymin, ymax) in self.wall_aabbs:
            closest_x = max(xmin, min(x, xmax))
            closest_y = max(ymin, min(y, ymax))
            dx = x - closest_x
            dy = y - closest_y
            if dx * dx + dy * dy < r2:
                return True
        return False

    def build_wall_markers(self, time_now):
        markers = []
        for i, (cx, cy, length, thickness, yaw) in enumerate(self.wall_segments):
            m = Marker()
            m.header.stamp    = time_now
            m.header.frame_id = 'map'
            m.ns     = 'walls'
            m.id     = 3000 + i
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = length
            m.scale.y = thickness
            m.scale.z = self.wall_height
            m.color.r = 0.5
            m.color.g = 0.5
            m.color.b = 0.5
            m.color.a = 1.0
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = self.wall_height / 2.0
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
        markers = []

        ns = 'tank1' if id_offset == 0 else 'tank2'
        body_z = 0.15  # matches the old TF broadcast for tank_body -> map

        q_body = euler_to_quaternion(yaw=body_theta, pitch=0, roll=0)
        # Turret's world yaw = body yaw + turret yaw (turret frame sits body_height/2 above body origin)
        q_turret = euler_to_quaternion(yaw=body_theta + turret_yaw, pitch=0, roll=0)
        turret_world_z = body_z + self.body_height / 2.0

        # Chassis
        chassis = Marker()
        chassis.header.stamp    = time_now
        chassis.header.frame_id = 'map'
        chassis.ns     = ns
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
        chassis.pose.position.x = body_x
        chassis.pose.position.y = body_y
        chassis.pose.position.z = body_z
        chassis.pose.orientation = q_body
        markers.append(chassis)

        # Wheels — local offset in body frame, rotated into world frame
        def add_wheel(marker_id, x_offset, y_side_multiplier):
            local_x = x_offset
            local_y = (self.body_width / 2.0 + self.wheel_width / 2.0) * y_side_multiplier
            local_z = -0.05
            wx, wy, wz = rotate_vec_by_quat(local_x, local_y, local_z, q_body)

            wheel = Marker()
            wheel.header.stamp    = time_now
            wheel.header.frame_id = 'map'
            wheel.ns     = ns
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
        turret_base.scale.x = self.turret_radius * 2
        turret_base.scale.y = self.turret_radius * 2
        turret_base.scale.z = self.turret_height
        turret_base.color.r = 0.2
        turret_base.color.g = 0.4
        turret_base.color.b = 0.2
        turret_base.color.a = 1.0
        turret_base.pose.position.x = body_x
        turret_base.pose.position.y = body_y
        turret_base.pose.position.z = turret_world_z + self.turret_height / 2.0
        turret_base.pose.orientation = q_turret
        markers.append(turret_base)

        # Cannon — local offset/orientation in turret frame, rotated into world
        cannon_local_x = (self.cannon_length / 2.0) * math.cos(cannon_pitch)
        cannon_local_z = self.turret_height + (self.cannon_length / 2.0) * math.sin(cannon_pitch)
        cx, cy, cz = rotate_vec_by_quat(cannon_local_x, 0.0, cannon_local_z, q_turret)

        cannon = Marker()
        cannon.header.stamp    = time_now
        cannon.header.frame_id = 'map'
        cannon.ns     = ns
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
        cannon.pose.position.x = body_x + cx
        cannon.pose.position.y = body_y + cy
        cannon.pose.position.z = turret_world_z + cz
        q_cannon_local = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + cannon_pitch, roll=0)
        cannon.pose.orientation = compose_quaternion(q_turret, q_cannon_local)
        markers.append(cannon)

        # Machine gun
        mg_mount_x = 0.05
        mg_mount_y = 0.15
        mg_mount_z_local = self.turret_height + self.mg_body_height / 2.0

        mbx, mby, mbz = rotate_vec_by_quat(mg_mount_x, mg_mount_y, mg_mount_z_local, q_turret)

        mg_body = Marker()
        mg_body.header.stamp    = time_now
        mg_body.header.frame_id = 'map'
        mg_body.ns     = ns
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
        mg_body.pose.position.x = body_x + mbx
        mg_body.pose.position.y = body_y + mby
        mg_body.pose.position.z = turret_world_z + mbz
        mg_body.pose.orientation = q_turret
        markers.append(mg_body)

        barrel_tip_offset_x = (self.mg_barrel_length / 2.0) * math.cos(self.mg_tilt)
        barrel_tip_offset_z = (self.mg_barrel_length / 2.0) * math.sin(self.mg_tilt)
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
        mg_barrel.scale.x = self.mg_barrel_radius * 2
        mg_barrel.scale.y = self.mg_barrel_radius * 2
        mg_barrel.scale.z = self.mg_barrel_length
        mg_barrel.color.r = 0.15
        mg_barrel.color.g = 0.15
        mg_barrel.color.b = 0.15
        mg_barrel.color.a = 1.0
        mg_barrel.pose.position.x = body_x + bbx
        mg_barrel.pose.position.y = body_y + bby
        mg_barrel.pose.position.z = turret_world_z + bbz
        q_barrel_local = euler_to_quaternion(yaw=0, pitch=-math.pi/2 + self.mg_tilt, roll=0)
        mg_barrel.pose.orientation = compose_quaternion(q_turret, q_barrel_local)
        markers.append(mg_barrel)

        amx = mg_mount_x
        amy = mg_mount_y + self.mg_body_width / 2.0 + 0.04
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

        if self.startup_grace_period > 0.0:
            self.startup_grace_period = max(0.0, self.startup_grace_period - dt)

        if not self.game_over:
            # --- Tank 1 movement (with wall collision) ---
            self.theta += self.angular_velocity * dt

            candidate_x = self.x + self.linear_velocity * math.cos(self.theta) * dt
            candidate_y = self.y + self.linear_velocity * math.sin(self.theta) * dt

            if not self._collides_with_walls(candidate_x, candidate_y):
                # No collision — move freely
                self.x = candidate_x
                self.y = candidate_y
            elif not self._collides_with_walls(candidate_x, self.y):
                # Blocked diagonally, but sliding along x alone is clear
                self.x = candidate_x
            elif not self._collides_with_walls(self.x, candidate_y):
                # Sliding along y alone is clear
                self.y = candidate_y
            # else: fully blocked — tank stays put this tick

            # --- Tank 2 autonomous movement + firing ---
            self.update_tank2_auto(dt)

        # --- Fire cooldown tick ---
        self.fire_cooldown = max(0.0, self.fire_cooldown - dt)
        self.mg_cooldown   = max(0.0, self.mg_cooldown   - dt)

        # --- Update projectiles and sparks (let existing shots finish even at game over) ---
        self.update_projectiles(dt)
        self.update_mg_projectiles(dt)
        self.update_sparks(dt)

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
        for m in self.build_tank_markers(time_now, self.x, self.y, self.theta,
                                          self.turret_yaw, self.cannon_pitch, id_offset=0):
            marker_array.markers.append(m)

        for m in self.build_tank_markers(time_now, self.x2, self.y2, self.theta2,
                                          self.turret_yaw2, self.cannon_pitch2, id_offset=300):
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


def main(args=None):
    rclpy.init(args=args)
    node = TankBodyVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()