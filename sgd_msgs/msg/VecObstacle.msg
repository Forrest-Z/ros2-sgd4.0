# Obstacle in vector space

# unique id for obstacle identification
uint32 id

# TODO: classification of obstacle

# confidence 0-255
uint8 confidence

# Pose
geometry_msgs/Pose pose

# Twist
geometry_msgs/TwistWithCovariance twist

# radius of obstacle: only used if vertices is not specified
uint32 radius

# vertices of obstacle outline
geometry_msgs/Point32[] vertices