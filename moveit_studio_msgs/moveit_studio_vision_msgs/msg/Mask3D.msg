# Message containing a 3D mask.
#
# This message may be used to represent an instance or semantic mask in the
# output message of a point cloud segmenter.

# The stamp and frame_id of the mask's header match those of the segmented point cloud's header.
std_msgs/Header header

# Indices of the points in the segmented point cloud that belong to this mask.
uint32[] point_indices

# Classification hypotheses sorted from most to least likely.
# If the mask was not classified, this field is empty.
ClassHypothesis[] class_hypotheses
