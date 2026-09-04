# Per-joint range bounds for a planning group. Joints not listed are unconstrained.
# Use the `joint_range_constraint` port on the planning Behaviors.
#
# YAML format on a Behavior port (top-level sequence, not a wrapped map — BT.CPP treats
# any port value of the form {key} as a blackboard reference):
#   []                                                              # empty (no constraint)
#   [{joint_name: rail, lower: 0.0, upper: 1.5}]
#   [{joint_name: rail, lower: 0.0, upper: 1.5}, {joint_name: shoulder, lower: -1.57, upper: 1.57}]
JointRange[] joint_ranges
