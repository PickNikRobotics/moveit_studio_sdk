# Optimization-related parameters for pro_rrt-based planning Behaviors (e.g. PlanToJointGoal).
# Bundled into a single message so future optimization knobs can be added without breaking
# existing Behavior Tree XMLs (new fields default to 0 / their ROS defaults).
#
# YAML format on a Behavior port (top-level sequence containing a single map — BT.CPP treats
# any port value of the form `{key}` as a blackboard reference, so wrapping the top level in
# `{...}` would be misinterpreted):
#   []                                                                       # empty (use defaults)
#   [{planning_seed_attempts: 5, optimization_iterations: 2000}]             # explicit values

# Number of RRT-Connect runs to attempt with different (Halton-spaced) seeds before picking the
# cheapest path. Higher values explore different homotopy classes and can find better initial
# routes that a single seed misses, at roughly linear extra cost (RRT-Connect is cheap relative
# to the optimization step). 0 is treated as 1 (single attempt, current behavior).
uint32 planning_seed_attempts

# Number of Informed RRT* refinement iterations to run after the planner finds a feasible path.
# The refined path is guaranteed to be no worse (in configuration-space length) than the
# unrefined one. 0 skips refinement. Iteration count rather than wall-clock budget gives
# reproducible results across machines; expected latency depends on scene complexity and is
# typically O(milliseconds per iteration).
uint32 optimization_iterations
