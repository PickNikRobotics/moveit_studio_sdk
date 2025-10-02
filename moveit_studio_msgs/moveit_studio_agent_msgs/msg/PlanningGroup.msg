# Name of the joint/planning group, as defined in the SRDF.
string name

# Indicates if an IK solver is defined for this group.
bool has_ik_solver

# Base frame of the planning group.
# If 'has_ik_solver' this is determined by that solver.
# Otherwise this is computed from the joint group.
string base_frame

# Tip frames of the planning group.
# If 'has_ik_solver' this is determined by that solver.
# Otherwise this is computed from the joint group.
string[] tip_frames

# The end-effectors connected to this planning group.
EndEffector[] end_effectors
