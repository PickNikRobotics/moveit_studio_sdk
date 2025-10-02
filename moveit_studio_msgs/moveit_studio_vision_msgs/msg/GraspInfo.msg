# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] graspable_faces

# Subframes. These correspond to affordances and their poses (relative to the object's pose).
ObjectSubframe[] subframes

# Optional class or name of the object type.
string object_type
