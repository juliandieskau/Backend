# Cannot directly send this one => no header
string    ectsname   # Name in ROS-Topic path

int8 TYPE_INTERVAL     = 0    # Aggregate for <interval> seconds
int8 TYPE_READINGS     = 1    # Aggregate <nreadings> together

int8 type

float32 interval              # in seconds, used if type == TYPE_INTERVAL
uint16  nreadings             # used if type == TYPE_READINGS

uint32  keep_amount           # amount of aggregated values that are kept

