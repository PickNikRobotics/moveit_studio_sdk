# This message describes a solid rigid object which a robot can grasp.

# The header defines the object's timestamp along with the base frame that the
# object's poses are relative to.
#std_msgs/Header header

# The name or classification of the object
string label
# The specific numeric ID of the object. id: 0 = default, -1 = delete
int32 id

# TransformStamped message defining the Pose and frames of the object.
# The frame_id is the frame the object is relative to.
# The child_id is the object's own reference frame, which subframes are relative to.
# The transform is the Pose of the object relative to the frame_id.
# The timestamp is when the transform for the object was last calculated.
geometry_msgs/TransformStamped transform

# The object's pose relative to the header frame.
# The poses of the surfaces, meshes, and primitives are defined relative to this pose.
geometry_msgs/PoseStamped pose # TODO remove.

# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] surfaces

# Primitives. A vector of solid geometric primitives which describe the shape of the object.
# The primitive_poses must be the same length, where each pose is relative to the transform.child_id
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# Meshes. A vector of meshes which describe the shape of the object.
# The mesh_poses must be the same length, where each pose is relative to the transform.child_id
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# Subframes. These correspond to affordances and their poses.
# The subframe_poses must be the same length, where each pose is relative to the transform.child_id
string[] subframe_names
geometry_msgs/Pose[] subframe_poses
