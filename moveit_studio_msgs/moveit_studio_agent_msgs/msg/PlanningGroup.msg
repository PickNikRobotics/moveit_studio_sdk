# Name of the planning group, as defined in the SRDF.
string name

# Indicates if an IK solver is defined for this group.
bool has_ik_solver

# Base frame of the planning group.
# Only filled in if the group has a kinematics solver.
string base_frame

# Tip frames of the planning group.
# Only filled in if the group has a kinematics solver.
string[] tip_frames

# The end-effectors connected to this planning group.
EndEffector[] end_effectors
