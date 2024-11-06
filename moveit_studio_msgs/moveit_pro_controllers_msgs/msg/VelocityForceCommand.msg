# The timestamp in the header is used to determine the age of the command.
# Commands older than the configured timeout will be rejected.
std_msgs/Header header

# The frame in which the command is given.
# Relative to the end-effector frame defined in the configs. Defaults to identity.
# This can be used to specify an offset with respect to the end-effector frame. Global control frames are not
# yet supported.
geometry_msgs/Transform control_frame

# Cartesian axes to control with velocity vs. force.
# A value of 'false' disables control of that axis, whereas a value of 'true' enables control.
# By default (empty), no axes will be controlled. Therefore the user must set at least one axis to control.
# These are updated once at the beginning of each stream of commands, i.e. they won't take effect if modified
# during execution.
# Given in the control frame (see 'control_frame' above).
moveit_pro_controllers_msgs/CartesianSelectionVector velocity_controlled_axes
moveit_pro_controllers_msgs/CartesianSelectionVector force_controlled_axes

# Gain to use in force-controlled axes.
# Multiplier on the force error to determine Cartesian velocity in the force-controlled axes.
# velocity_on_force_axis = wrench_gain * (wrench_desired - wrench_actual).
# The gain is updated once at the beginning of each stream of commands, i.e. it won't take effect if modified during
# execution.
# Needs to be set, otherwise it will default to 0.0 (no motion).
float64 wrench_gain

# Desired twist (in the velocity-controlled axes) and wrench (in the force-controlled axes).
# These can be modified at any time, and the controller will do its best to achieve them.
# Given in the control frame.
geometry_msgs/Twist twist
geometry_msgs/Wrench wrench

# Nullspace parameter to be optionally used for secondary control goal.
# Empty array will be ignored, otherwise array length must match number of degrees-of-freedom.
# These can be modified at any time, and the controller should do its best to minimize joint-space costs if a nullspace controller is implemented.
float64[] nullspace_component
