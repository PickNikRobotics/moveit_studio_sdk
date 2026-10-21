# Constrains how a tip link must follow a Cartesian path during path inverse kinematics.
# Pass this to the `cartesian_constraint` port of PlanCartesianPath / SetupMTCPathIK.
#
# This message is intentionally minimal today (a single constraint type) but is a message rather
# than a bool so it can grow with additional constraint fields (e.g. per-axis masks or tolerances)
# without breaking the Behavior ports that consume it.
#
# YAML format on a Behavior port (top-level sequence with a single map — BT.CPP treats any port
# value of the form {key} as a blackboard reference, so a bare top-level map cannot be used):
#   []                                       # empty: defaults to POSITION_ONLY
#   [{constraint_type: position_only}]
#   [{constraint_type: position_and_orientation}]

# Constrain only the tip translation (x, y, z); the solver is free to choose the orientation.
uint8 POSITION_ONLY=0

# Constrain both the tip translation and orientation (full 6-DOF).
uint8 POSITION_AND_ORIENTATION=1

uint8 constraint_type
