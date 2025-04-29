# QuadStatePub.msg

Header header
string child_frame_id
uint8 drone_id                        # My drone id
geometry_msgs/PoseStamped pose        # My pose in my world frame
float32[12] pose_cov                   # Upper triangular matrix of attitude and position covariance
float32[3] gyr                        # the unbiased angular velocity (body frame) of the body origin
float32[3] vel                        # the estimated velocity (global frame) of the body origin
float32[3] world_to_gravity_deg       # the estimated euler angles from world to gravity frame in degrees
float64 swarmlio_start_time           # the time when swarm-lio starts
bool degenerated                      # whether the drone is degenerated
ObserveTeammate[] teammate            # Teammates' states observed by me



