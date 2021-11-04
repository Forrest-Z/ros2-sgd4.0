# Message to control lighting of shared guide dog

# define some modes
uint8 FILL = 0
uint8 BLINK = 1
uint8 WAFT = 2
uint8 RUN = 3

# define left and right as ints
uint8 RIGHT = 0
uint8 LEFT = 1
uint8 BOTH = 2

std_msgs/Header header

# Set mode
uint8 mode

# select strip
uint8 strip

# the rgb color code
uint8[] rgb