# Message containing a collection of 2D masks.
# 
# Image segmenters may use this message to represent their output.

# The header's timestamp is the acquisition time of the segmented image.
# The header's frame ID is the optical frame of the camera.
std_msgs/Header header

# Segmented masks.
moveit_studio_vision_msgs/Mask2D[] masks
