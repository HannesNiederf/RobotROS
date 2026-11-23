# Spec: softwarespezifikation.md section 5.1.
# Pose convention for STORAGE stations: see spez_aenderung_01_rueckwaerts.md D-33
# (pose = robot position with cart, front pointing out of the box).
string KIND_STORAGE="STORAGE"
string KIND_DOCK="DOCK"

uint32 id
string name
string kind
geometry_msgs/Pose2D pose
float32 approach_distance
bool enabled
bool occupied
string cart_name
