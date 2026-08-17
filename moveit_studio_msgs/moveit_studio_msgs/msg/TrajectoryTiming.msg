# Selects the algorithm used to assign timestamps to a planned Cartesian path, plus its parameters.
# Bundled into a single message so future timing knobs can be added without breaking existing
# Behavior Tree XMLs (new fields default to 0 / their ROS defaults).
#
# YAML format on a Behavior port (top-level sequence containing a single map — BT.CPP treats any
# port value of the form `{key}` as a blackboard reference, so wrapping the top level in `{...}`
# would be misinterpreted):
#   []                                                                          # empty (use defaults: TIME_OPTIMAL)
#   [{mode: time_optimal, velocity_scale_factor: 1.0, acceleration_scale_factor: 1.0,
#     trajectory_sampling_rate: 100}]
#   [{mode: cartesian_trapezoidal, max_translational_velocity: 0.1, max_rotational_velocity: 0.5,
#     max_translational_acceleration: 0.5, max_rotational_acceleration: 2.0, trajectory_sampling_rate: 100}]

# Timing algorithm to use. Defaults to TIME_OPTIMAL (0).
uint8 mode
uint8 TIME_OPTIMAL=0           # Time-Optimal Trajectory Generation (TOTG); time-optimal under joint limits.
uint8 CARTESIAN_TRAPEZOIDAL=1  # Constant-speed trapezoidal velocity profile in Cartesian space.

# Output trajectory sample rate, in Hz. Used by both modes. Leave at 0 to use the default rate (100 Hz).
uint32 trajectory_sampling_rate

# TIME_OPTIMAL parameters.
float64 velocity_scale_factor      # In [0, 1]; scales the robot's joint velocity limits. Leave at 0 for full scaling.
float64 acceleration_scale_factor  # In [0, 1]; scales the robot's joint acceleration limits. Leave at 0 for full scaling.

# CARTESIAN_TRAPEZOIDAL parameters.
float64 max_translational_velocity      # Cruise translational speed, in m/s.
float64 max_rotational_velocity         # Cruise rotational speed, in rad/s.
float64 max_translational_acceleration  # Translational ramp acceleration, in m/s^2.
float64 max_rotational_acceleration     # Rotational ramp acceleration, in rad/s^2.
