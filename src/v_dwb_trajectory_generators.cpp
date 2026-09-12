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

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <vector>

#include "dwb_core/trajectory_generator.hpp"
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace f_dwa_controller
{
namespace
{

std::vector<double> fixed_velocity_samples(
  double current, const double minimum, const double maximum,
  const double acceleration, const double deceleration,
  const double duration, const int count)
{
  if (count < 1 || !std::isfinite(current)) {
    throw std::invalid_argument("V-DWA requires a positive sample count and finite velocity");
  }
  current = std::clamp(current, minimum, maximum);
  const double low = dwb_plugins::projectVelocity(
    current, acceleration, deceleration, duration, minimum);
  const double high = dwb_plugins::projectVelocity(
    current, acceleration, deceleration, duration, maximum);
  if (low == high) {
    return {low};
  }
  if (count == 1) {
    return {std::clamp(current, low, high)};
  }
  std::vector<double> samples(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    samples[index] = low + (high - low) *
      static_cast<double>(index) / static_cast<double>(count - 1);
  }
  samples.back() = high;
  // Keep zero within the requested budget. Upstream inserts an extra sample
  // when the regular grid straddles zero, giving 159/160 instead of 150.
  // Preserve both reachable endpoints and replace the nearest interior point.
  if (count >= 3 && low < 0.0 && high > 0.0) {
    const auto closest = std::min_element(
      samples.begin() + 1, samples.end() - 1,
      [](const double a, const double b) {return std::abs(a) < std::abs(b);});
    *closest = 0.0;
  }
  return samples;
}

class FixedCountVelocityIterator : public dwb_plugins::XYThetaIterator
{
public:
  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & current, const double duration) override
  {
    auto limits = kinematics_handler_->getKinematics();
    const auto x = fixed_velocity_samples(
      current.x, limits.getMinX(), limits.getMaxX(), limits.getAccX(),
      limits.getDecelX(), duration, vx_samples_);
    const auto y = fixed_velocity_samples(
      current.y, limits.getMinY(), limits.getMaxY(), limits.getAccY(),
      limits.getDecelY(), duration, vy_samples_);
    const auto theta = fixed_velocity_samples(
      current.theta, limits.getMinTheta(), limits.getMaxTheta(),
      limits.getAccTheta(), limits.getDecelTheta(), duration, vtheta_samples_);
    commands_.clear();
    commands_.reserve(x.size() * y.size() * theta.size());
    for (const double linear_x : x) {
      for (const double linear_y : y) {
        for (const double angular : theta) {
          // Zero is a member of the reachable bank. The swept-body and
          // dynamically feasible stopping gates still decide admissibility.
          if ((linear_x == 0.0 && linear_y == 0.0 && angular == 0.0) ||
            limits.isValidSpeed(linear_x, linear_y, angular))
          {
            nav_2d_msgs::msg::Twist2D command;
            command.x = linear_x;
            command.y = linear_y;
            command.theta = angular;
            commands_.push_back(command);
          }
        }
      }
    }
    next_ = 0u;
  }

  bool hasMoreTwists() override {return next_ < commands_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return commands_.at(next_++);}

private:
  std::vector<nav_2d_msgs::msg::Twist2D> commands_;
  std::size_t next_{0u};
};

}  // namespace

void VLimitedAccelTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  velocity_iterator_ = std::make_shared<FixedCountVelocityIterator>();
  velocity_iterator_->initialize(node, kinematics_handler_, plugin_name_);
  sampling_node_ = node;
}

void VLimitedAccelTrajectoryGenerator::reset()
{
  const auto node = sampling_node_.lock();
  if (!node) {
    throw std::runtime_error("V-DWA sampling node is unavailable");
  }
  for (const auto * key : {"vx_samples", "vtheta_samples"}) {
    if (node->get_parameter(plugin_name_ + "." + key).as_int() < 1) {
      throw std::invalid_argument(std::string("V-DWA ") + key + " must be positive");
    }
  }
  dwb_plugins::LimitedAccelGenerator::reset();
  // The upstream iterator caches sample counts at initialization, not per cycle.
  initializeIterator(node);
}

void VLimitedAccelTrajectoryGenerator::set_planning_snapshot(
  std::shared_ptr<const PlanningSnapshot> snapshot)
{
  planning_snapshot_ = std::move(snapshot);
}

nav_2d_msgs::msg::Twist2D VLimitedAccelTrajectoryGenerator::current_command_velocity(
  const nav_2d_msgs::msg::Twist2D & current_velocity) const
{
  if (planning_snapshot_ && planning_snapshot_->valid &&
    planning_snapshot_->activation_state.native_command_velocity_valid)
  {
    return planning_snapshot_->activation_state.native_command_velocity;
  }
  return current_velocity;
}

void VLimitedAccelTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  dwb_plugins::LimitedAccelGenerator::startNewIteration(
    current_command_velocity(current_velocity));
}

void VStandardTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  velocity_iterator_ = std::make_shared<FixedCountVelocityIterator>();
  velocity_iterator_->initialize(node, kinematics_handler_, plugin_name_);
}

}  // namespace f_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::VLimitedAccelTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(
  f_dwa_controller::VStandardTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
