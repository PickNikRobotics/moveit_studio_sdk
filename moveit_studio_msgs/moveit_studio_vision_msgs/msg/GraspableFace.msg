# This message describes a flat region on the surface of an object.

# A polygon which represents a flat surface on an object.
# Only x and y fields of the points are used in the polygon.
# The points in the polygon are ordered counter-clockwise.
geometry_msgs/Polygon vertices

# A pose which orients the polygon relative to an externally-defined frame of reference
# The normal vector of the face points along the Z+ axis.
geometry_msgs/Pose pose
