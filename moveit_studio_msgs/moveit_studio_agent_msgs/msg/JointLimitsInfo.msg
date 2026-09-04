# Per-joint limits for the Joint Monitor view pane.
#
# Two sets of limits are reported so the UI can show both and explain how they
# differ: the limits declared in the URDF, and the effective limits after the
# joint_limits.yaml overrides are applied to the MoveIt RobotModel. The URDF has
# no acceleration field, so urdf.has_acceleration_limits is always false.

# Joint name.
string name

# True for revolute joints (incl. continuous), whose values are angular (radians); false for prismatic
# joints, whose values are linear (meters). Lets the UI label positions/velocities/accelerations in the
# correct unit (degrees vs meters).
bool is_revolute

# Limits declared in the URDF <limit> tag (position lower/upper, velocity).
moveit_msgs/JointLimits urdf

# Effective limits after joint_limits.yaml overrides are applied to the RobotModel.
moveit_msgs/JointLimits moveit
