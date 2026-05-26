# Admittance parameters.
# These are the parameters of a mass-spring-damper model that determine
# the admittance behavior on each of the Cartesian axes of each tip.
# Each of the individual mass, spring, and damper vectors have 6 elements, one for each Cartesian axis.
# They are given in the order: [X, Y, Z, RX, RY, RZ].
# Use carefully. Certain choices may lead to unstable behavior.
float64[6] mass
float64[6] stiffness
float64[6] damping
bool[6] selected_axes  # Axes in the control frame to control with admittance.
# Joint space damping. If there are multiple tips and corresponding chains in the Admittance problem, and the chains
# share joints, then the joint_damping must match in the AdmittanceParameters that correspond to the overlapping joints.
float64 joint_damping

# End-effector frame, mass, and center of gravity (CoG) position.
string end_effector_frame
float64 end_effector_mass
float64[3] end_effector_cog  # X,Y,Z center of gravity in the end-effector frame.
geometry_msgs/Pose ee_pose_control  # Control frame w.r.t. end-effector frame. If empty, will default to identity pose.

# The absolute force or torque threshold value for each Cartesian axis at which
# the controller will stop executing if exceeded.
# If the absolute value of an axis of the measured FTS wrench exceeds the corresponding
# threshold, the trajectory goal is aborted with 'error_code' set to
# FORCE_TORQUE_THRESHOLD_EXCEEDED.
# If empty, default threshold values from configs will be used.
float64[] absolute_force_torque_threshold
