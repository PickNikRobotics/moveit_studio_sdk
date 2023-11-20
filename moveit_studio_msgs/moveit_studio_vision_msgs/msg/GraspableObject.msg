# This message describes a solid rigid object which a robot can grasp.

# The header defines the object's timestamp along with the base frame to use
# when creating a TF to the object instance.
std_msgs/Header header

# The name or identifier of the object
# This is what is used as the `child_frame_id` when creating a TF to this object.
string id

# The pose of the object relative to the frame specified in the header. This is 
# used to create the transform to the object, from which all subframe poses are relative to.
geometry_msgs/Pose pose

# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] surfaces

# Primitives. A vector of solid geometric primitives which describe the shape of the object.
shape_msgs/SolidPrimitive[] primitives
# The primitive_poses must be the same length as primitives, where each pose is relative to the objects pose.
geometry_msgs/Pose[] primitive_poses

# Meshes. A vector of meshes which describe the shape of the object.
shape_msgs/Mesh[] meshes
# The mesh_poses must be the same length as meshes, where each pose is relative to the objects pose.
geometry_msgs/Pose[] mesh_poses

# Subframes. These correspond to affordances and their poses (relative to the object's pose).
ObjectSubframe[] subframes

# The four coeffcients of a plane estimated from all object points.
# The plane is relative to the header's frame ID.
float32[] main_plane_coefficients

