# TankState.msg — Custom message for a single tank's full state
#
# Contains position, orientation, turret angles, health, and velocity.

# Header with timestamp and frame_id
std_msgs/Header header

# Tank body pose (2D ground plane + heading)
float64 x            # X position in the map frame (meters)
float64 y            # Y position in the map frame (meters)
float64 theta        # Body heading / yaw angle (radians)

# Turret state (relative to body)
float64 turret_yaw   # Turret yaw relative to the body (radians)
float64 cannon_pitch # Cannon pitch relative to the turret (radians)

# Velocity
float64 linear_velocity   # Forward speed (m/s)
float64 angular_velocity  # Turn rate (rad/s)

# Health
float64 health       # Current HP
float64 max_health   # Maximum HP

# Combat status
bool is_alive        # True if this tank is still in the game
string tank_name     # Identifier (e.g. "tank1", "tank2")
