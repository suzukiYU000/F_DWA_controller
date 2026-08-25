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

#ifndef F_DWA_CONTROLLER__VELOCITY_RESPONSE_MODEL_HPP_
#define F_DWA_CONTROLLER__VELOCITY_RESPONSE_MODEL_HPP_

#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

namespace f_dwa_controller
{

struct AxisVelocityResponseModel
{
  double dead_time_seconds{0.0};
  double time_constant_seconds{0.0};
  double steady_state_gain{1.0};
};

struct VelocityResponsePrediction
{
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D velocity;
  std::vector<geometry_msgs::msg::Pose2D> trajectory;
  bool valid{false};
};

bool valid_velocity_response_model(
  const AxisVelocityResponseModel & model);

double predict_axis_velocity(
  double observed_velocity,
  double dispatched_command,
  double dispatch_age_seconds,
  double prediction_seconds,
  const AxisVelocityResponseModel & model);

VelocityResponsePrediction predict_velocity_response(
  const geometry_msgs::msg::Pose2D & observed_pose,
  const nav_2d_msgs::msg::Twist2D & observed_velocity,
  const nav_2d_msgs::msg::Twist2D & dispatched_command,
  double dispatch_age_seconds,
  double prediction_seconds,
  double maximum_integration_step_seconds,
  const AxisVelocityResponseModel & linear_model,
  const AxisVelocityResponseModel & angular_model);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__VELOCITY_RESPONSE_MODEL_HPP_
