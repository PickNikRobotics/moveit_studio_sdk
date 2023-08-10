# This message describes a solid rigid object which a robot can grasp.

# The header defines the base frame of the object's pose as well as the object's timestamp.
std_msgs/Header header

# The object's pose relative to the header frame.
# The poses of the surfaces, meshes, and primitives are defined relative to this pose.
geometry_msgs/Pose pose

# The id of the object
string id

# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] surfaces

# A vector of solid geometric primitives which describe the shape of the object.
shape_msgs/SolidPrimitive[] primitives

# The size of this vector must be equal to the size of the vector of primitives.
# Each of these poses is relative to the pose defined in the object's `pose` field.
geometry_msgs/Pose[] primitive_poses

# A vector of meshes which describe the shape of the object.
shape_msgs/Mesh[] meshes

# The size of this vector must be equal to the size of the vector of meshes.
# Each of these poses is relative to the pose defined in the object's `pose` field.
geometry_msgs/Pose[] mesh_poses
