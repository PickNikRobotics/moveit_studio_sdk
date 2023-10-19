# This message describes a solid rigid object which a robot can grasp.

# The id and label of the object
int32 id
string label

# The object's pose relative to specified frame.
# The poses of the surfaces, meshes, and primitives are defined relative to this pose.
geometry_msgs/PoseStamped pose

# A solid primitive which represents the maximum extent of the object.
# The origin of this primitive is coincident with the object's pose.
shape_msgs/SolidPrimitive bounding_volume

# A vector of surfaces which represents flat faces on the object.
moveit_studio_vision_msgs/GraspableFace[] surfaces

# A vector of "part" messages. Each includes either a primitive shape or a mesh, along with a pose
# and a label.
moveit_studio_vision_msgs/ObjectPartPrimitive[] primitive_parts
moveit_studio_vision_msgs/ObjectPartMesh[] mesh_parts
