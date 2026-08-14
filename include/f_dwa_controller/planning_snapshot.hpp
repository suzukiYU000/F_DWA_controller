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

#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "rclcpp/time.hpp"

namespace f_dwa_controller
{

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
  double linear_acceleration{0.0};
  double angular_acceleration{0.0};
  std::vector<double> linear_fir_history;
  std::vector<double> angular_fir_history;
  rclcpp::Time activation_time;
  bool native_state_valid{true};
};

// A PlanningSnapshot is assembled once at the start of a control cycle and is
// then shared as const. It contains only robot-observable dispatch history and
// the configured nominal future delay; simulator ground truth and future jitter
// samples are deliberately excluded.
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
