// Copyright 2026 YTLab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef F_DWA_CONTROLLER__ISSUED_COMMAND_LEDGER_HPP_
#define F_DWA_CONTROLLER__ISSUED_COMMAND_LEDGER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "rclcpp/time.hpp"

namespace f_dwa_controller
{

struct IssuedCommandLedgerEntry
{
  rclcpp::Time issued_at;
  uint64_t issued_steady_time_ns{0};
  nav_2d_msgs::msg::Twist2D command;
  bool is_controller_failure_stop{false};
};

// Return only a command that could already have reached the transport at the
// transport-reception epoch. A delayed callback must never consume a later,
// numerically identical Controller result.
std::optional<std::size_t> find_eligible_command_index(
  const std::deque<IssuedCommandLedgerEntry> & commands,
  uint64_t received_steady_time_ns,
  const nav_2d_msgs::msg::Twist2D & dispatched,
  double tolerance = 1.0e-9);

std::size_t eligible_command_prefix_size(
  const std::deque<IssuedCommandLedgerEntry> & commands,
  uint64_t received_steady_time_ns);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__ISSUED_COMMAND_LEDGER_HPP_
