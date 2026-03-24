std_msgs/Header header

# Identity
string robot_ns
uint8 team

# Basic health/combat status
float32 hp
float32 ammo
bool alive
bool in_combat

# Kinematics/state estimate
geometry_msgs/Pose pose
geometry_msgs/Twist twist

# Optional discrete behavior state (0 idle, 1 patrol, 2 attack, 3 retreat)
uint8 mode
