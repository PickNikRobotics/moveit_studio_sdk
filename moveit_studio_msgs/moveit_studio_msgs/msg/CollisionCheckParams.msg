# Collision-validation parameters for the pro_rrt-based planning Behaviors (e.g. PlanToJointGoal) and,
# for the padding fields only, the jog Behaviors (PoseJog, JointJog):
# how densely motions are collision-checked during the search, and whether the final trajectory is
# re-validated before it is returned. Bundled into a single message so future validation knobs can
# be added without breaking existing Behavior Tree XMLs (new fields default to 0 / their ROS
# defaults).
#
# Presence convention: a field for which 0 is a meaningful value carries an explicit override_*
# bool (link_padding, world_padding); a field for which 0 is meaningless reads 0 as "use the default"
# (sanity_check_max_attempts). New fields must pick one of these two idioms, not coin a third.
#
# YAML format on a Behavior port (top-level sequence containing a single map — BT.CPP treats
# any port value of the form `{key}` as a blackboard reference, so wrapping the top level in
# `{...}` would be misinterpreted):
#   []                                                                      # empty (use defaults)
#   [{workspace_step: 0.1, override_scene_link_padding: true, link_padding: 0.005, override_scene_world_padding: true, world_padding: 0.01, sanity_collision_check: true, sanity_check_max_attempts: 4}]

# Workspace resolution used to subdivide motions for collision validation, in meters. When
# positive, every motion segment the planner validates — tree extensions and candidate shortcuts —
# is sampled so that no point of the robot (attached bodies included) sweeps more than this
# distance between consecutive collision checks, on top of the planner's joint-space step. 0.1 is
# the benchmarked starting point: it roughly halves collision tunneling through thin obstacles at
# the cost of more collision checks per segment. Leave at 0 (default) to disable the workspace-referred
# refinement and validate at the joint-space step alone.
float64 workspace_step

# When true, link_padding below overrides the planning scene's link padding for this Behavior —
# an explicit override, including to 0 (no padding). When false (the default), link_padding must
# stay at 0 and the Behavior leaves in place the link padding of the planning scene it is handed —
# for MTC stages, the scene produced upstream in the task — which is 0 on a fresh MoveIt Pro instance. A nonzero link_padding with this flag false is rejected at tick time, so
# a padding the author wrote can never be silently ignored. In the YAML port form, an engaged
# flag also requires the link_padding key to be written explicitly (0 for no padding), so a
# misspelled key cannot silently engage a zero-padding override.
bool override_scene_link_padding

# Padding in meters applied to the robot's collision geometry (attached bodies included) for
# planning and collision checking, used only while override_scene_link_padding is true; 0 then
# means no padding. While the Behavior's deprecated `link_padding` input port is set, that port's
# value takes precedence over this field and the Behavior logs a deprecation warning; unset the
# port to migrate.
float64 link_padding

# When true, world_padding below overrides the planning scene's world object padding for this Behavior, including to 0 (no padding). When false (the default), world_padding must stay at 0 and the Behavior leaves in place the world object padding of the planning scene it is handed. For MTC stages that is the scene produced upstream in the task. PlanToJointGoal and the jog Behaviors build their scene from a planning scene message, which carries no world object padding, so they apply none. The Behavior rejects a nonzero world_padding while this flag is false, and in the YAML port form an engaged flag needs world_padding written explicitly.
bool override_scene_world_padding

# Padding in meters applied to the collision geometry of every world object for planning and collision checking, used only while override_scene_world_padding is true; 0 then means no padding. It replaces any per-object padding the scene carries, and adds to link_padding where a padded link approaches a padded object. Octree and plane shapes cannot be padded, so this adds no margin around an octomap.
float64 world_padding

# When true, the planned trajectory (corner blends included) is collision-checked sample by sample
# before it is returned, as a last-resort guard against collision tunneling; the check runs against
# the actual collision geometry, without padding or scaling (a contact only inside the padding margin
# does not count), and its resolution follows the output trajectory sampling rate. If a collision is found,
# planning is repeated with a different seed, within the attempt budget below; if every attempt
# still collides, planning fails naming the colliding bodies instead of returning the colliding
# trajectory. Off by default: the check costs roughly one collision check per trajectory sample,
# and each extra attempt costs a full planning call.
bool sanity_collision_check

# Total planning attempts allowed when sanity_collision_check is enabled: the initial attempt plus
# up to N-1 replans with different seeds. 1 keeps the check but fails on the first detected
# collision without replanning. Leave at 0 for the default budget (4: the initial attempt plus 3
# replans). Values above 100 are rejected — each attempt is a full planning call, so larger values
# are almost certainly a typo. Ignored when sanity_collision_check is false.
uint32 sanity_check_max_attempts
