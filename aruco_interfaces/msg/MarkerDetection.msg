# One ArUco marker seen in one camera frame.
# Units follow REP 103: metres and radians.

# Id in the dictionary the detector was configured with.
int32 id

# Marker centre and orientation in the camera optical frame (x right, y down, z forward).
# The marker's own z axis points out of its printed side.
geometry_msgs/Pose pose

# Distance from the camera centre to the marker centre.
float64 distance

# Angle between the marker's normal and the line of sight to the camera: 0 is head-on.
float64 viewing_angle

# RMS distance in pixels between the detected corners and where the pose puts them.
float64 reprojection_error

# How alike the two poses a flat square allows fit its corners: the smaller of their summed
# squared reprojection errors over the larger. Near 0 one pose fits clearly better; near 1 the
# corners cannot tell them apart, and the pose may be the mirrored one.
float64 ambiguity
