# This message describes a solid rigid object which a robot can grasp.

# The id and label of the object
# id: 0 = default, -1 = delete
int32 id
string label
#string id

# The object's pose relative to specified frame.
# The poses of the surfaces, meshes, and primitives are defined relative to this pose.
geometry_msgs/PoseStamped pose

# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] surfaces

# Primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# Meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# Subframes. These correspond to affordances and their poses
string[] subframe_names
geometry_msgs/Pose[] subframe_poses
