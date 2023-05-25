# Array of obstacles in vector space

std_msgs/Header header

# MetaData for the map
nav_msgs/MapMetaData info

# Pose of the object with respect to the frame_id specified in the header
geometry_msgs/Pose pose

# obstacle Array
sgd_msgs/VecObstacle[] obstacles