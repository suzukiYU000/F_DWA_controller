# Observable software dispatch time. This is not a motor-actuation timestamp.
std_msgs/Header header
# Time at which this exact command entered the delayed transport FIFO. Unlike
# header.stamp, this precedes the sampled 60--80 ms delay and therefore
# disambiguates later Controller results with the same numeric value.
builtin_interfaces/Time received_at
# Monotonic-clock epoch at FIFO reception for sequenced commands, or at the
# initial/reset/emergency boundary for an unsequenced event. This is the
# authoritative causal provenance for matching an applied command to a
# Controller result. ROS time remains above for diagnostics, but /clock samples
# are not synchronized between processes in accelerated simulation.
uint64 received_steady_time_ns
geometry_msgs/Twist command
uint64 sequence_id
# False denotes the initial/reset zero ledger boundary.
bool has_sequence
