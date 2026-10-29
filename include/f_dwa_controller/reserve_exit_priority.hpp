// Optional selection preference; physical rollout and stop gates remain external.
#pragma once
#include <cmath>

namespace f_dwa_controller
{
inline bool reserve_exit_priority(
  const bool enabled, const double initial_clearance,
  const double terminal_clearance, const double margin)
{
  // No minimum along the path: a rectangular turn may approach temporarily.
  // Use an absolute reserve target, not a moving "no worse than now" target.
  return enabled && std::isfinite(initial_clearance) &&
         std::isfinite(terminal_clearance) && std::isfinite(margin) &&
         margin > 0.0 && initial_clearance < margin &&
         terminal_clearance >= margin;
}
}  // namespace f_dwa_controller
