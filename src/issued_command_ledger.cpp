// Copyright 2026 YTLab
// Licensed under the MIT License.

#include "f_dwa_controller/issued_command_ledger.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>

namespace f_dwa_controller
{

namespace
{

bool was_issued_no_later_than(
  const IssuedCommandLedgerEntry & entry,
  const uint64_t received_steady_time_ns)
{
  return entry.issued_steady_time_ns != 0u &&
         received_steady_time_ns != 0u &&
         entry.issued_steady_time_ns <= received_steady_time_ns;
}

bool commands_match_with_tolerance(
  const nav_2d_msgs::msg::Twist2D & expected,
  const nav_2d_msgs::msg::Twist2D & observed,
  const double tolerance)
{
  return std::isfinite(tolerance) && tolerance >= 0.0 &&
         std::abs(expected.x - observed.x) <= tolerance &&
         std::abs(expected.y - observed.y) <= tolerance &&
         std::abs(expected.theta - observed.theta) <= tolerance;
}

}  // namespace

std::size_t eligible_command_prefix_size(
  const std::deque<IssuedCommandLedgerEntry> & commands,
  const uint64_t received_steady_time_ns)
{
  const auto first_future = std::find_if_not(
    commands.begin(), commands.end(),
    [received_steady_time_ns](const IssuedCommandLedgerEntry & entry) {
      return was_issued_no_later_than(entry, received_steady_time_ns);
    });
  return static_cast<std::size_t>(
    std::distance(commands.begin(), first_future));
}

std::optional<std::size_t> find_eligible_command_index(
  const std::deque<IssuedCommandLedgerEntry> & commands,
  const uint64_t received_steady_time_ns,
  const nav_2d_msgs::msg::Twist2D & dispatched,
  const double tolerance)
{
  const std::size_t eligible_count =
    eligible_command_prefix_size(commands, received_steady_time_ns);
  const auto eligible_end = std::next(
    commands.begin(), static_cast<std::ptrdiff_t>(eligible_count));
  const auto match = std::find_if(
    commands.begin(), eligible_end,
    [&dispatched, tolerance](const IssuedCommandLedgerEntry & entry) {
      return commands_match_with_tolerance(
        entry.command, dispatched, tolerance);
    });
  if (match == eligible_end) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(commands.begin(), match));
}

}  // namespace f_dwa_controller
