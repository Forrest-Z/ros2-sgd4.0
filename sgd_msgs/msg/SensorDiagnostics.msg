# Message to send additional information about sensors

std_msgs/Header header

# define sensor states
uint8 UNCONFIGURED = 0
uint8 PART_UNCONF = 1
uint8 MOST_CONFIG = 2
uint8 FULL_CONFIG = 3

uint8 state