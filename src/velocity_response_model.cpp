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

#include "f_dwa_controller/velocity_response_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace f_dwa_controller
{

namespace
{

bool finite_twist(const nav_2d_msgs::msg::Twist2D & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.theta);
}

bool finite_pose(const geometry_msgs::msg::Pose2D & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.theta);
}

}  // namespace

bool valid_velocity_response_model(
  const AxisVelocityResponseModel & model)
{
  return std::isfinite(model.dead_time_seconds) &&
         model.dead_time_seconds >= 0.0 &&
         std::isfinite(model.time_constant_seconds) &&
         model.time_constant_seconds > 0.0 &&
         std::isfinite(model.steady_state_gain) &&
         model.steady_state_gain > 0.0;
}

double predict_axis_velocity(
  const double observed_velocity,
  const double dispatched_command,
  const double dispatch_age_seconds,
  const double prediction_seconds,
  const AxisVelocityResponseModel & model)
{
  if (!std::isfinite(observed_velocity) ||
    !std::isfinite(dispatched_command) ||
    !std::isfinite(dispatch_age_seconds) || dispatch_age_seconds < 0.0 ||
    !std::isfinite(prediction_seconds) || prediction_seconds < 0.0 ||
    !valid_velocity_response_model(model))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double remaining_dead_time = std::max(
    0.0, model.dead_time_seconds - dispatch_age_seconds);
  const double active_response_time = std::max(
    0.0, prediction_seconds - remaining_dead_time);
  if (active_response_time <= 0.0) {
    return observed_velocity;
  }

  const double target_velocity =
    model.steady_state_gain * dispatched_command;
  const double retained_fraction = std::exp(
    -active_response_time / model.time_constant_seconds);
  return target_velocity +
         (observed_velocity - target_velocity) * retained_fraction;
}

VelocityResponsePrediction predict_velocity_response(
  const geometry_msgs::msg::Pose2D & observed_pose,
  const nav_2d_msgs::msg::Twist2D & observed_velocity,
  const nav_2d_msgs::msg::Twist2D & dispatched_command,
  const double dispatch_age_seconds,
  const double prediction_seconds,
  const double maximum_integration_step_seconds,
  const AxisVelocityResponseModel & linear_model,
  const AxisVelocityResponseModel & angular_model)
{
  VelocityResponsePrediction prediction;
  prediction.pose = observed_pose;
  prediction.velocity = observed_velocity;
  if (!finite_pose(observed_pose) || !finite_twist(observed_velocity) ||
    !finite_twist(dispatched_command) ||
    !std::isfinite(dispatch_age_seconds) || dispatch_age_seconds < 0.0 ||
    !std::isfinite(prediction_seconds) || prediction_seconds < 0.0 ||
    !std::isfinite(maximum_integration_step_seconds) ||
    maximum_integration_step_seconds <= 0.0 ||
    !valid_velocity_response_model(linear_model) ||
    !valid_velocity_response_model(angular_model))
  {
    return prediction;
  }

  prediction.trajectory.push_back(observed_pose);
  if (prediction_seconds == 0.0) {
    prediction.valid = true;
    return prediction;
  }

  const int step_count = std::max(
    1, static_cast<int>(
      std::ceil(prediction_seconds / maximum_integration_step_seconds)));
  const double time_step = prediction_seconds / step_count;
  geometry_msgs::msg::Pose2D pose = observed_pose;
  nav_2d_msgs::msg::Twist2D velocity = observed_velocity;
  for (int step_index = 0; step_index < step_count; ++step_index) {
    const double elapsed_before = step_index * time_step;
    const double elapsed_after = (step_index + 1) * time_step;
    nav_2d_msgs::msg::Twist2D next_velocity;
    next_velocity.x = predict_axis_velocity(
      observed_velocity.x, dispatched_command.x, dispatch_age_seconds,
      elapsed_after, linear_model);
    next_velocity.y = 0.0;
    next_velocity.theta = predict_axis_velocity(
      observed_velocity.theta, dispatched_command.theta, dispatch_age_seconds,
      elapsed_after, angular_model);
    const double previous_linear_velocity = predict_axis_velocity(
      observed_velocity.x, dispatched_command.x, dispatch_age_seconds,
      elapsed_before, linear_model);
    const double previous_angular_velocity = predict_axis_velocity(
      observed_velocity.theta, dispatched_command.theta, dispatch_age_seconds,
      elapsed_before, angular_model);
    const double average_linear_velocity =
      0.5 * (previous_linear_velocity + next_velocity.x);
    const double average_angular_velocity =
      0.5 * (previous_angular_velocity + next_velocity.theta);
    const double heading_change = average_angular_velocity * time_step;
    const double midpoint_heading = pose.theta + 0.5 * heading_change;
    pose.x += average_linear_velocity * std::cos(midpoint_heading) * time_step;
    pose.y += average_linear_velocity * std::sin(midpoint_heading) * time_step;
    pose.theta += heading_change;
    prediction.trajectory.push_back(pose);
    velocity = next_velocity;
  }

  prediction.pose = pose;
  prediction.velocity = velocity;
  prediction.valid = finite_pose(prediction.pose) &&
    finite_twist(prediction.velocity);
  return prediction;
}

}  // namespace f_dwa_controller
