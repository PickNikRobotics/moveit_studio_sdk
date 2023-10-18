# Message containing a 2D mask.
#
# This message may be used to represent an instance or semantic mask in the
# output message of an image segmenter.

# The position of the top-left corner of the mask within the segmented image. 
# (0, 0) is the top-left corner of the segmented image.
int32 x
int32 y

# A single plane image with the mask pixels.
# The image pixels can be encoded as:
#   - Integer: only non-zero pixels belong to the mask.
#   - Floating point: each pixel has a probability of belonging to the mask.
# The stamp and frame_id of the mask image header match those of the segmented image header.
sensor_msgs/Image pixels
