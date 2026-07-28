# Observable software dispatch time. This is not a motor-actuation timestamp.
std_msgs/Header header
geometry_msgs/Twist command
uint64 sequence_id
# False denotes the initial/reset zero ledger boundary.
bool has_sequence
