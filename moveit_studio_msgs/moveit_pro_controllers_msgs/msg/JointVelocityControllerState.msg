# This message contains information about the internal state of the JointVelocityController.

# Timestamps associated to this state.
std_msgs/Header header

# Joint names associated to the controller.
# The order of the names is the same as the order of the joint positions, velocities and accelerations.
string[] joint_names

# The reference joint-space positions, velocities and accelerations, as computed by the setpoint generator.
float64[] reference_joint_positions
float64[] reference_joint_velocities
float64[] reference_joint_accelerations

# The current joint-space positions, velocities and accelerations, reported by the HW interface, or estimated internally.
float64[] joint_positions
float64[] joint_velocities
float64[] joint_accelerations
