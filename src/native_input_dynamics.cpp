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

#include "f_dwa_controller/native_input_dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace f_dwa_controller
{

namespace
{

constexpr double kFeasibilityTolerance = 1.0e-12;

bool state_and_limits_are_valid(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step)
{
  return std::isfinite(state.velocity) &&
         std::isfinite(state.acceleration) &&
         std::isfinite(limits.velocity_min) &&
         std::isfinite(limits.velocity_max) &&
         std::isfinite(limits.acceleration_min) &&
         std::isfinite(limits.acceleration_max) &&
         std::isfinite(limits.native_input_min) &&
         std::isfinite(limits.native_input_max) &&
         std::isfinite(time_step) &&
         time_step > 0.0 &&
         limits.velocity_min <= limits.velocity_max &&
         limits.acceleration_min <= limits.acceleration_max &&
         limits.native_input_min <= limits.native_input_max;
}

FeasibleInterval make_interval(const double lower, const double upper)
{
  FeasibleInterval interval;
  interval.lower = lower;
  interval.upper = upper;
  interval.feasible = lower <= upper + kFeasibilityTolerance;
  if (interval.feasible && interval.lower > interval.upper) {
    const double midpoint = 0.5 * (interval.lower + interval.upper);
    interval.lower = midpoint;
    interval.upper = midpoint;
  }
  return interval;
}

}  // namespace

FeasibleInterval acceleration_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step)
{
  if (!state_and_limits_are_valid(state, limits, time_step)) {
    return {};
  }

  const double velocity_lower =
    (limits.velocity_min - state.velocity) / time_step;
  const double velocity_upper =
    (limits.velocity_max - state.velocity) / time_step;
  return make_interval(
    std::max(
      {limits.native_input_min, limits.acceleration_min, velocity_lower}),
    std::min(
      {limits.native_input_max, limits.acceleration_max, velocity_upper}));
}

FeasibleInterval jerk_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step)
{
  if (!state_and_limits_are_valid(state, limits, time_step)) {
    return {};
  }

  const double time_step_squared = time_step * time_step;
  const double acceleration_lower =
    (limits.acceleration_min - state.acceleration) / time_step;
  const double acceleration_upper =
    (limits.acceleration_max - state.acceleration) / time_step;
  const double velocity_lower =
    (limits.velocity_min - state.velocity -
    state.acceleration * time_step) / time_step_squared;
  const double velocity_upper =
    (limits.velocity_max - state.velocity -
    state.acceleration * time_step) / time_step_squared;
  return make_interval(
    std::max(
      {limits.native_input_min, acceleration_lower, velocity_lower}),
    std::min(
      {limits.native_input_max, acceleration_upper, velocity_upper}));
}

FeasibleInterval held_acceleration_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step,
  const int remaining_steps)
{
  if (!state_and_limits_are_valid(state, limits, time_step) ||
    remaining_steps <= 0)
  {
    return {};
  }

  double lower =
    std::max(limits.native_input_min, limits.acceleration_min);
  double upper =
    std::min(limits.native_input_max, limits.acceleration_max);
  for (int step = 1; step <= remaining_steps; ++step) {
    const double duration = time_step * static_cast<double>(step);
    lower = std::max(
      lower, (limits.velocity_min - state.velocity) / duration);
    upper = std::min(
      upper, (limits.velocity_max - state.velocity) / duration);
  }
  return make_interval(lower, upper);
}

FeasibleInterval held_jerk_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step,
  const int remaining_steps)
{
  if (!state_and_limits_are_valid(state, limits, time_step) ||
    remaining_steps <= 0)
  {
    return {};
  }

  double lower = limits.native_input_min;
  double upper = limits.native_input_max;
  for (int step = 1; step <= remaining_steps; ++step) {
    const double step_count = static_cast<double>(step);
    const double acceleration_coefficient = time_step * step_count;
    lower = std::max(
      lower,
      (limits.acceleration_min - state.acceleration) /
      acceleration_coefficient);
    upper = std::min(
      upper,
      (limits.acceleration_max - state.acceleration) /
      acceleration_coefficient);

    const double velocity_response =
      state.velocity + step_count * state.acceleration * time_step;
    const double velocity_coefficient =
      time_step * time_step * step_count * (step_count + 1.0) * 0.5;
    lower = std::max(
      lower,
      (limits.velocity_min - velocity_response) / velocity_coefficient);
    upper = std::min(
      upper,
      (limits.velocity_max - velocity_response) / velocity_coefficient);
  }
  return make_interval(lower, upper);
}

ProjectedAxisStep project_acceleration_step(
  const AxisState & state,
  const AxisLimits & limits,
  const double native_input_reference,
  const double time_step)
{
  const FeasibleInterval interval =
    acceleration_input_interval(state, limits, time_step);
  if (!interval.feasible || !std::isfinite(native_input_reference)) {
    return {};
  }

  ProjectedAxisStep result;
  result.applied_native_input =
    std::clamp(native_input_reference, interval.lower, interval.upper);
  result.state.acceleration = result.applied_native_input;
  result.state.velocity =
    state.velocity + result.state.acceleration * time_step;
  result.feasible = true;
  return result;
}

ProjectedAxisStep project_jerk_step(
  const AxisState & state,
  const AxisLimits & limits,
  const double native_input_reference,
  const double time_step)
{
  const FeasibleInterval interval =
    jerk_input_interval(state, limits, time_step);
  if (!interval.feasible || !std::isfinite(native_input_reference)) {
    return {};
  }

  ProjectedAxisStep result;
  result.applied_native_input =
    std::clamp(native_input_reference, interval.lower, interval.upper);
  result.state.acceleration =
    state.acceleration + result.applied_native_input * time_step;
  result.state.velocity =
    state.velocity + result.state.acceleration * time_step;
  result.feasible = true;
  return result;
}

ProjectedAxisStep project_held_acceleration_step(
  const AxisState & state,
  const AxisLimits & limits,
  const double native_input_reference,
  const double time_step,
  const int remaining_steps)
{
  const FeasibleInterval interval =
    held_acceleration_input_interval(
    state, limits, time_step, remaining_steps);
  if (!interval.feasible || !std::isfinite(native_input_reference)) {
    return {};
  }

  return project_acceleration_step(
    state, limits,
    std::clamp(native_input_reference, interval.lower, interval.upper),
    time_step);
}

ProjectedAxisStep project_held_jerk_step(
  const AxisState & state,
  const AxisLimits & limits,
  const double native_input_reference,
  const double time_step,
  const int remaining_steps)
{
  const FeasibleInterval interval =
    held_jerk_input_interval(state, limits, time_step, remaining_steps);
  if (!interval.feasible || !std::isfinite(native_input_reference)) {
    return {};
  }

  return project_jerk_step(
    state, limits,
    std::clamp(native_input_reference, interval.lower, interval.upper),
    time_step);
}

std::vector<double> uniform_samples(
  const FeasibleInterval & interval,
  const int sample_count)
{
  if (!interval.feasible || sample_count <= 0) {
    return {};
  }

  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(sample_count));
  if (sample_count == 1) {
    samples.push_back(interval.lower);
    return samples;
  }

  for (int index = 0; index < sample_count; ++index) {
    const double ratio =
      static_cast<double>(index) / static_cast<double>(sample_count - 1);
    samples.push_back(
      interval.lower + (interval.upper - interval.lower) * ratio);
  }
  return samples;
}

}  // namespace f_dwa_controller
