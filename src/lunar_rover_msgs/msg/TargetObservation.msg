uint8 SURFACE_UNKNOWN=0
uint8 SURFACE_GROUND=1
uint8 SURFACE_WALL=2

std_msgs/Header header
string observation_id
string class_id
float32 confidence
geometry_msgs/Pose pose
geometry_msgs/Vector3 surface_normal
geometry_msgs/Vector3 dimensions
uint8 surface_type
bool pose_is_precise
string source
