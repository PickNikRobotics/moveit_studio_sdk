# This message describes a constituent Part of an Object, defined in 
# primitive shapes or meshes given their offset pose.
string label
# This pose is relative to the pose defined in the parent object's `pose` field.
geometry_msgs/Pose pose

# A mesh message which describes the shape of the object part.
shape_msgs/Mesh mesh
