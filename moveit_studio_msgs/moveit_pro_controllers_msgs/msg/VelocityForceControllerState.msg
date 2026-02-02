# This message contains information about the internal state of the VelocityForceController.

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

# The reference Cartesian pose, velocity and acceleration, as computed by the setpoint generator.
geometry_msgs/Pose reference_cartesian_pose
geometry_msgs/Twist reference_cartesian_velocity
geometry_msgs/Twist reference_cartesian_acceleration

# The current Cartesian pose and velocity, reported by the HW interface, or estimated internally.
geometry_msgs/Pose cartesian_pose
geometry_msgs/Twist cartesian_velocity
