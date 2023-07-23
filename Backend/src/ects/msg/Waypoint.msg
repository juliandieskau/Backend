string name                # Human readable name for waypoint

# copied from iosb_nav_msgs/Waypoint
geometry_msgs/Pose2D pose  # position of waypoint center and desired heading of robot

float32 radius             # radius of waypoint [m]

float32 heading_accuracy   # desired accuracy of heading [rad]

bool use_heading           # true: desired heading is considered
                           # false: desired heading is ignored

float32 wait_time          # when waypoint has been reached, wait for specified time [s]
