# Heartbeat timestamps of the Objective Server node.
builtin_interfaces/Time process_timestamp
builtin_interfaces/Time ros_timestamp

# Name and status descriptor of the current objective tree.
string objective_name

# Possible objective_status values.
int8 OBJECTIVE_STATUS_UNKNOWN = 0
int8 OBJECTIVE_STATUS_IDLE = 1
int8 OBJECTIVE_STATUS_RUNNING = 2
int8 OBJECTIVE_STATUS_SUCCESS = 3
int8 OBJECTIVE_STATUS_FAILURE = 4
int8 OBJECTIVE_STATUS_CANCELLED = 5

# The status of the objective.
int8 objective_status
