# Detection3DStamped.msg
std_msgs/Header header
geometry_msgs/Point position
int32 class_id                  # YOLO class id (e.g., 0..4)
string class_name
string drone_ns
float32 score

