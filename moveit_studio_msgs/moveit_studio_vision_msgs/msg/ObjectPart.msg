# This message describes a constituent Part of an Object, defined in 
# primitive shapes or meshes given their offset pose.
string label
# This pose is relative to the pose defined in the parent object's `pose` field.
geometry_msgs/Pose pose

# Either a mesh or a primitive.
# A vector of solid geometric primitives which describe the shape of the object.
shape_msgs/SolidPrimitive[<=1] primitive
# A vector of meshes which describe the shape of the object.
shape_msgs/Mesh[<=1] mesh
