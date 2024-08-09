# Admittance parameters.
# These are the parameters of a mass-spring-damper model that determines
# the admittance behavior on each of the Cartesian axes.
# Use carefully. Certain choices may lead to unstable behavior.
float64[] mass
float64[] stiffness
float64[] damping
float64 joint_damping

geometry_msgs/Pose ee_pose_control   # Control frame w.r.t. end-effector frame.
bool[] selected_axes                 # Axes in the control frame to control with admittance.
