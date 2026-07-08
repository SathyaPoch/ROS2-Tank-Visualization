# ROS2 TANK VISUALIZER 
## A ROS2 package for 3D tank robot visualization with keyboard teleperation and turret control 

--------------------
### Tank Combat Arena 
Tank battle with independent turret rotation.

### Custom Message:
TankCommand.msg (track_left_speed, track_right_speed, turret_angle, fire_cannon,
fire_machinegun). 

### Keyboard Control: 
WASD for tank movement, Arrow keys for turret, Space
(cannon), Ctrl (machine gun). 

### Visualization:
rviz2 with separate body and turret markers.

### Current in branch shooting_canon
i = stop
k = shoot machine gun
space = shoot cannon