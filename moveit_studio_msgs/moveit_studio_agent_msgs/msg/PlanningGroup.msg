# Name of the planning group, as defined in the SRDF.
string name

# Indicates if an IK solver is defined for this group
# and thus, if it can be used by servo
bool has_ik_solver

# Base frame of the planning group.
# Only filled in if the group has a kinematics solver.
string base_frame

# Tip frames of the planning group.
# Only filled in if the group has a kinematics solver.
string[] tip_frames
