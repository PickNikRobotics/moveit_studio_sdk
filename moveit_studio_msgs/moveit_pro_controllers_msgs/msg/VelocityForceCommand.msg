# The timestamp in the header is used to determine the age of the command.
# Commands older than the configured timeout will be rejected.
std_msgs/Header header

# Name of the robot-model link the command is expressed in, before applying the 'control_frame_offset'
# below. Must be a link that exists in the robot model (not an arbitrary TF frame).
# Leave empty (the default) to express the command in the controller's configured 'ee_frame',
# which is the original end-effector-relative behavior. Set to a fixed frame (e.g. the world or
# base link) to send world-relative references that stay fixed as the end-effector rotates.
string control_frame_id

# Orientation offset applied on top of 'control_frame_id'.
# Defaults to identity. Use this to express the command in a frame rotated with respect to
# 'control_frame_id'. Only the rotation is used: the controller re-expresses the commanded twist/wrench
# by orientation alone. The translation is ignored as an implementation detail -- the controller does not
# model any lever-arm coupling between an offset control frame and the end-effector.
geometry_msgs/Transform control_frame_offset

# DEPRECATED: use 'control_frame_offset' instead. This field has the same meaning (an orientation offset
# applied on top of 'control_frame_id') and is still honored, but the controller logs a deprecation warning
# when it is set and 'control_frame_offset' is not. It will be removed in the next breaking release.
geometry_msgs/Transform control_frame

# Cartesian axes to control with velocity vs. force.
# A value of 'false' disables control of that axis, whereas a value of 'true' enables control.
# By default (empty), no axes will be controlled. Therefore the user must set at least one axis to control.
# These are updated once at the beginning of each stream of commands, i.e. they won't take effect if modified
# during execution.
# Given in the control frame (see 'control_frame_id' and 'control_frame_offset' above).
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
