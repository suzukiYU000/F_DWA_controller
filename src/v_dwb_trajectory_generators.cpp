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

#include "f_dwa_controller/v_dwb_trajectory_generators.hpp"

#include <utility>

#include "dwb_core/trajectory_generator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{

void VLimitedAccelTrajectoryGenerator::set_planning_snapshot(
  std::shared_ptr<const PlanningSnapshot> snapshot)
{
  planning_snapshot_ = std::move(snapshot);
}

void VLimitedAccelTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  if (planning_snapshot_ && planning_snapshot_->valid &&
    planning_snapshot_->activation_state.native_command_velocity_valid)
  {
    dwb_plugins::LimitedAccelGenerator::startNewIteration(
      planning_snapshot_->activation_state.native_command_velocity);
    return;
  }
  dwb_plugins::LimitedAccelGenerator::startNewIteration(current_velocity);
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::VLimitedAccelTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::VStandardTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
