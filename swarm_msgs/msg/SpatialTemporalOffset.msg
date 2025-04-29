# SpatialTemporalOffset.msg
uint8 teammate_id # The drone_id of teammate
float32[3] rot_deg # rotation in euler angles (degrees)
float32[4] rot_quaternion # rotation in quaternion: w x y z
float32[3] trans # translation (meters)
float32 time_offset # (seconds)