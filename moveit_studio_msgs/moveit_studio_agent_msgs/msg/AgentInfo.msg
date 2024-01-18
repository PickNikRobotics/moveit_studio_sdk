# The agent version as stored in the package.xml file.
string version

# The state field contains reported state of the Objective Server.

# UNCONFIGURED means that the state field was not correctly set when the Objective Server created the AgentInfo message.
# It does not correspond to an operational state of the Objective Server.
int8 UNCONFIGURED = 0

# IDLE means that the Objective Server is not performing an Objective.
int8 IDLE = 1

# OBJECTIVE_IN_PROGRESS means that the Objective Server is currently performing an Objective.
int8 OBJECTIVE_IN_PROGRESS = 2

# This will be set to UNCONFIGURED, IDLE, or OBJECTIVE_IN_PROGRESS.
int8 state

# The name of the currently running Objective
string current_objective_name

# Information about the license: expiration date, type, etc.
moveit_pro_agent_msgs/LicenseInfo license
