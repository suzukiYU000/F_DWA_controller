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
