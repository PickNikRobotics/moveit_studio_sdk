# Heartbeat timestamps of the Objective Server node.
builtin_interfaces/Time process_timestamp
builtin_interfaces/Time ros_timestamp

# Name and status descriptor of the current objective tree.
string objective_name

# Possible objective_status values.
string OBJECTIVE_STATUS_UNKNOWN = Unknown
string OBJECTIVE_STATUS_IDLE = Idle
string OBJECTIVE_STATUS_RUNNING = Running
string OBJECTIVE_STATUS_SUCCESS = Success
string OBJECTIVE_STATUS_FAILURE = Failure
string OBJECTIVE_STATUS_CANCELLED = Cancelled

# The status of the objective.
string objective_status
