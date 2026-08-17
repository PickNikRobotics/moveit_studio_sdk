# Planar orientation constraint for a single link. Keeps the link's initial orientation during the
# motion, allowing only rotation about the world-vertical (Z) axis. `angle_tolerance` is the maximum
# off-plane angle, in radians. The consuming planning Behavior requires it to be greater than zero
# and at most pi, and fails the plan otherwise.
string link_name
float64 angle_tolerance
