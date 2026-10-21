# Planar orientation constraints for one or more links, used by the ProRRT planning Behaviors.
# Each listed link keeps its initial orientation during the motion, allowing only rotation about the
# world-vertical (Z) axis, within its own `angle_tolerance`. Links not listed are unconstrained.
# An empty list means no orientation constraint. Each link may appear at most once; listing the same
# link more than once is rejected. Use the `orientation_constraint` port on the planning Behaviors.
#
# YAML format on a Behavior port (top-level sequence, not a wrapped map — BT.CPP treats
# any port value of the form {key} as a blackboard reference):
#   []                                                       # empty (no constraint)
#   [{link_name: grasp_link, angle_tolerance: 0.1}]
#   [{link_name: grasp_link, angle_tolerance: 0.1}, {link_name: tool0, angle_tolerance: 0.05}]
LinkOrientationConstraint[] link_constraints
