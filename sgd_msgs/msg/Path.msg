# An array of poses that represents a Path for a robot to follow.

# Robot is operating normal
uint8 NORMAL = 0
uint8 NORMAL_SLOW = 1
uint8 TURN_LEFT = 2
uint8 TURN_RIGHT = 3
uint8 FAILURE = 9


# Indicates the frame_id of the path.
std_msgs/Header header

# Array of poses to follow.
geometry_msgs/PoseStamped[] poses

# Array of states
uint8 state