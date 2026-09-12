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

#ifndef F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_
#define F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_

#include <memory>

#include "dwb_plugins/limited_accel_generator.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "f_dwa_controller/planning_snapshot.hpp"

namespace f_dwa_controller
{

// V-DWA samples velocity commands inside the acceleration-reachable window of
// the command that will be active when the newly planned command reaches the
// robot-facing transport.  That is distinct from the lagging physical velocity
// used to predict the activation pose.  Keeping both states explicit prevents
// response lag from granting a fictitious one-cycle acceleration reversal.
class VLimitedAccelTrajectoryGenerator
  : public dwb_plugins::LimitedAccelGenerator
{
public:
  void set_planning_snapshot(
    std::shared_ptr<const PlanningSnapshot> snapshot);
  nav_2d_msgs::msg::Twist2D current_command_velocity(
    const nav_2d_msgs::msg::Twist2D & current_velocity) const;
  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  void reset() override;

protected:
  void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & node) override;

private:
  nav2_util::LifecycleNode::WeakPtr sampling_node_;
  std::shared_ptr<const PlanningSnapshot> planning_snapshot_;
};

// Preserve the standard rollout while sharing the fixed sample-count iterator.
class VStandardTrajectoryGenerator
  : public dwb_plugins::StandardTrajectoryGenerator
{
protected:
  void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & node) override;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_
