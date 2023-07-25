# Message type that represents a Behavior parameter.

# The names of the Behaviors that the parameter applies to.
# Currently, each parameter always applies to exactly one Behavior.
string[] Behavior_namespaces

# The description of the Behavior Parameter.
moveit_studio_sdk_msgs/BehaviorParameterDescription description

# The value of the Behavior Parameter, derived from the `description` field.
# Only one value should be set.
string string_value
geometry_msgs/PoseStamped pose_value
float64 double_value
int32 int_value
bool bool_value
sensor_msgs/JointState joint_state_value
