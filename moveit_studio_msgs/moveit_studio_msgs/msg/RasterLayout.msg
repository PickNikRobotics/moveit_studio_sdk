# Layout of a serpentine raster over a region, for coverage path Behaviors such as GenerateSurfaceCoveragePath.
# Bundled into a single message so the layout can grow without breaking existing Behavior Tree XMLs: a field left
# out of the port value is 0. For the two spacings, 0 is never a valid distance and is the fixed sentinel for "use the
# Behavior's default"; every other field's 0 is a real value.
#
# YAML format on a Behavior port (top-level sequence containing a single map — BT.CPP treats any
# port value of the form `{key}` as a blackboard reference, so wrapping the top level in `{...}`
# would be misinterpreted):
#   []                                                                               # defaults
#   [{line_spacing: 0.05, point_spacing: 0.02, margin: 0.0, line_direction: along_x}]

# Distance between adjacent raster lines, in meters. 0 selects the Behavior's default.
float64 line_spacing

# Sampling distance along each raster line, in meters. 0 selects the Behavior's default.
float64 point_spacing

# Inset from the region's edges on all sides, in meters. 0 is a real inset of nothing, not a default sentinel.
float64 margin

# Axis of the region the raster lines run along; the lines are stepped along the other axis, and every
# waypoint's X axis points along the lines. Defaults to ALONG_X (0).
uint8 line_direction
uint8 ALONG_X=0  # Lines run along the region's X axis.
uint8 ALONG_Y=1  # Lines run along the region's Y axis.
