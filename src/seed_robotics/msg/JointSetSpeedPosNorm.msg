# Normalized speed/position command for a single joint.
# target_pos: wrist in radians [_WRIST_LOWER, _WRIST_UPPER]; finger closedness [0.0, 1.0].
# target_speed: fraction of max raw speed [0.0, 1.0]; negative keeps the last known speed.
string name
float32 target_pos
float32 target_speed
