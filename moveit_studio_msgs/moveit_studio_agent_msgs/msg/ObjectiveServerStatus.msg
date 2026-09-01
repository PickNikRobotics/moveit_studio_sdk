# Heartbeat timestamps of the Objective Server node.
builtin_interfaces/Time process_timestamp
builtin_interfaces/Time ros_timestamp

# Name and status descriptor of the current objective tree.
string objective_name

# Runtime execution identifier for the active objective tree, or the most recently completed
# execution reported in heartbeats. Empty when no execution has been started or retained.
string execution_id

# Possible objective_status values.
string OBJECTIVE_STATUS_UNKNOWN = Unknown
string OBJECTIVE_STATUS_IDLE = Idle
string OBJECTIVE_STATUS_RUNNING = Running
string OBJECTIVE_STATUS_SUCCESS = Success
string OBJECTIVE_STATUS_FAILURE = Failure
string OBJECTIVE_STATUS_CANCELLED = Cancelled
string OBJECTIVE_STATUS_LOADING = Loading

# The status of the objective.
string objective_status
