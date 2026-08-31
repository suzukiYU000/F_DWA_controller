# Start time of the controller plugin invocation in the ROS clock.
std_msgs/Header header

# Monotonic sequence number assigned by the instrumented controller instance.
uint64 sequence

# Controller plugin ID selected by the FollowPath goal.
string controller_id

# Wall-clock time spent inside computeVelocityCommands().
builtin_interfaces/Duration duration

# True when computeVelocityCommands() returned without throwing.
bool success

# Populated only when success is false. These fields preserve the controller's
# exact failure diagnosis at the failed cycle instead of relying on a later,
# potentially stale LocalPlanEvaluation sample.
string failure_type
string failure_message
