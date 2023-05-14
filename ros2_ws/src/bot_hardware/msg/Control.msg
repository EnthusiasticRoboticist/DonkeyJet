int64 INVALID=-2

std_msgs/Header header
# A value between -1 and 1, with -1 indicating full throttle in backward and 1 indicating full throttle in the forward.
# Use INVALID if there is no value for it.
float64 throttle
# A value between -1 and 1, with -1 indicating full left steering and 1 indicating full right steering.
# Use INVALID if there is no value for it.
float64 steering