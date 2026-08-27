// Copyright (c) 2026 suzukiYU000
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef F_DWA_CONTROLLER__PLANNING_SNAPSHOT_HPP_
#define F_DWA_CONTROLLER__PLANNING_SNAPSHOT_HPP_

#include <cmath>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "rclcpp/time.hpp"

namespace f_dwa_controller
{

/**
 * @brief Age of a robot-facing dispatch already observed by this process.
 *
 * Gazebo /clock and DDS callbacks can be delivered in opposite executor order,
 * so an event observed in the current cycle may carry a header a few
 * milliseconds ahead of the Controller's latest clock sample. Such bounded
 * skew means zero elapsed plant-response time; it is not an unobserved future
 * command. A stamp farther ahead than one configured control period remains
 * invalid and cannot enter the planning state.
 */
inline std::optional<double> observed_dispatch_age_seconds(
  const rclcpp::Time & measurement_time,
  const rclcpp::Time & observed_dispatch_time,
  const double maximum_future_skew_seconds)
{
  if (!std::isfinite(maximum_future_skew_seconds) ||
    maximum_future_skew_seconds < 0.0)
  {
    return std::nullopt;
  }
  if (observed_dispatch_time > measurement_time) {
    const double future_skew_seconds =
      (observed_dispatch_time - measurement_time).seconds();
    if (!std::isfinite(future_skew_seconds) ||
      future_skew_seconds > maximum_future_skew_seconds + 1.0e-12)
    {
      return std::nullopt;
    }
    return 0.0;
  }
  const double age_seconds =
    (measurement_time - observed_dispatch_time).seconds();
  if (!std::isfinite(age_seconds) || age_seconds < 0.0) {
    return std::nullopt;
  }
  return age_seconds;
}

struct ScheduledCommand
{
  rclcpp::Time activation_time;
  nav_2d_msgs::msg::Twist2D command;
  bool is_controller_failure_stop{false};
};

struct ActivationState
{
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  // The identified plant-response preview above is a physical velocity.  A
  // native A/J/F window must instead continue from the command state already
  // accepted by the robot-facing FIFO, including commands that will dispatch
  // before activation.  Keep both states explicit so response lag cannot be
  // mistaken for additional native acceleration authority.
  nav_2d_msgs::msg::Twist2D native_command_velocity;
  double linear_acceleration{0.0};
  double angular_acceleration{0.0};
  std::vector<double> linear_fir_history;
  std::vector<double> angular_fir_history;
  rclcpp::Time activation_time;
  bool native_command_velocity_valid{false};
  bool native_state_valid{true};
};

// A PlanningSnapshot is assembled once at the start of a control cycle and is
// then shared as const. current_state is the measured physical state;
// activation_state is its transport replay or identified-response prediction.
// Native command velocity, acceleration, and FIR fields remain command-state
// memory correlated with robot-facing dispatches. Simulator ground truth and
// unobserved future jitter samples are deliberately excluded.
struct PlanningSnapshot
{
  rclcpp::Time measurement_time;
  rclcpp::Time activation_time;
  ActivationState current_state;
  ActivationState activation_state;
  std::vector<ScheduledCommand> committed_commands;
  std::vector<geometry_msgs::msg::Pose2D> delay_trajectory;
  bool dispatch_state_observed{false};
  bool valid{false};
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__PLANNING_SNAPSHOT_HPP_
