# Status of a behavior tree breakpoint, published on /breakpoint_encountered
# while a BreakpointSubscriber is paused and on /breakpoint_resumed when it
# unpauses (continue, abort, or halt).

# The topic the BreakpointSubscriber is listening on for the resume/abort Bool.
# Used by the UI as a unique key to track active breakpoints and to publish
# back to when the operator clicks Resume.
string topic

# Optional human-readable message authored on the breakpoint (e.g. "Waiting
# for operator to verify gripper alignment"). Empty when no message was
# provided. Only meaningful on /breakpoint_encountered.
string message
