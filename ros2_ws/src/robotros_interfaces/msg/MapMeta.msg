# Spec: softwarespezifikation.md section 5.3 (topic /web/map_meta).
# The spec names "resolution, origin, size"; the exact field types are chosen here.
float32 resolution        # m per cell
geometry_msgs/Pose2D origin
uint32 width              # cells
uint32 height             # cells
