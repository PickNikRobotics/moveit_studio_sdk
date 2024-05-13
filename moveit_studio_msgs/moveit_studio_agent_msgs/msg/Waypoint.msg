# Unique name of the waypoint
string name

# Description of the waypoint.
string description

# A flag indicating whether this is a favorite waypoint.
bool favorite

# Robot arm joint state when the waypoint was created
sensor_msgs/JointState joint_state

# Robot arm joint state when the waypoint was created
sensor_msgs/MultiDOFJointState multi_dof_joint_state

# Associated groups
string[] joint_group_names
