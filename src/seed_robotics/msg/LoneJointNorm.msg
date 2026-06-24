# Normalized view of a single joint.
# Wrist joints: positions in radians within [_WRIST_LOWER, _WRIST_UPPER].
# Finger joints: positions as closedness in [0.0 (open), 1.0 (closed)].
# Speeds: signed fraction of the joint's max raw speed, in [-1.0, 1.0].
string name
uint8 bus_id
float32 target_position
float32 present_position
float32 target_speed
float32 present_speed
