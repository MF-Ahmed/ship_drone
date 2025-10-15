# TrackedObstacle.msg
std_msgs/Header header  # For timestamp and frame_id
int32 id                # Track ID
geometry_msgs/Point position
float64[9] covariance   # Flattened 3x3 covariance matrix

int32 class_id                  # YOLO class id (e.g., 0..4)
string drone_ns                 # e.g., "drone1", "drone2", ...
float32 score                   # optional detector confidence
string class_name 
