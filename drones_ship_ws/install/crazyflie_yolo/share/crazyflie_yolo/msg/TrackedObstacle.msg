# TrackedObstacle.msg
std_msgs/Header header  # For timestamp and frame_id
int32 id                # Track ID
geometry_msgs/Point position
float64[9] covariance   # Flattened 3x3 covariance matrix
