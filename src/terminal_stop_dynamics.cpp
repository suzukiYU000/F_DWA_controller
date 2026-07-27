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

#include "f_dwa_controller/terminal_stop_dynamics.hpp"

#include <algorithm>
#include <cmath>

#include "f_dwa_controller/fir_input_dynamics.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr double kBoundTightening = 2.0e-7;

bool stop_problem_is_valid(
  const AxisState & state,
  const AxisLimits & limits,
  const double time_step,
  const int maximum_steps,
  const double stop_velocity_threshold)
{
  return std::isfinite(state.velocity) &&
         std::isfinite(state.acceleration) &&
         std::isfinite(time_step) &&
         time_step > 0.0 &&
         maximum_steps > 0 &&
         std::isfinite(stop_velocity_threshold) &&
         stop_velocity_threshold > 0.0 &&
         limits.velocity_min <= 0.0 &&
         limits.velocity_max >= 0.0;
}

AxisLimits directional_limits(
  const AxisLimits & limits,
  const bool positive_direction)
{
  AxisLimits directional = limits;
  if (positive_direction) {
    directional.velocity_min = 0.0;
  } else {
    directional.velocity_max = 0.0;
  }
  directional.velocity_min += kBoundTightening;
  directional.velocity_max -= kBoundTightening;
  directional.acceleration_min += kBoundTightening;
  directional.acceleration_max -= kBoundTightening;
  directional.native_input_min += kBoundTightening;
  directional.native_input_max -= kBoundTightening;
  return directional;
}

double jerk_release_velocity_loss(
  const double signed_acceleration,
  const double signed_release_jerk,
  const double time_step)
{
  if (signed_acceleration >= 0.0 ||
    signed_release_jerk <= 0.0 ||
    time_step <= 0.0)
  {
    return 0.0;
  }

  double acceleration = signed_acceleration;
  double velocity_loss = 0.0;
  while (acceleration < 0.0) {
    const double jerk =
      std::min(signed_release_jerk, -acceleration / time_step);
    acceleration += time_step * jerk;
    velocity_loss += std::max(0.0, -time_step * acceleration);
  }
  return velocity_loss;
}

void mark_terminal(StopSequence & sequence)
{
  sequence.feasible = true;
  sequence.terminal_state_cleared = true;
}

}  // namespace

StopSequence generate_acceleration_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const double time_step,
  const int maximum_steps,
  const double stop_velocity_threshold)
{
  StopSequence sequence;
  if (!stop_problem_is_valid(
      initial_state, limits, time_step, maximum_steps,
      stop_velocity_threshold))
  {
    return sequence;
  }
  if (std::abs(initial_state.velocity) <= stop_velocity_threshold) {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  const bool positive_direction = initial_state.velocity > 0.0;
  const AxisLimits stop_limits =
    directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const double requested_acceleration =
      positive_direction ? stop_limits.native_input_min :
      stop_limits.native_input_max;
    const ProjectedAxisStep step =
      project_acceleration_step(
      state, stop_limits, requested_acceleration, time_step);
    if (!step.feasible) {
      return sequence;
    }
    sequence.native_inputs.push_back(step.applied_native_input);
    sequence.states.push_back(step.state);
    state = step.state;
    if (std::abs(state.velocity) <= stop_velocity_threshold) {
      mark_terminal(sequence);
      return sequence;
    }
  }
  return sequence;
}

StopSequence generate_jerk_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const double time_step,
  const int maximum_steps,
  const double stop_velocity_threshold)
{
  StopSequence sequence;
  if (!stop_problem_is_valid(
      initial_state, limits, time_step, maximum_steps,
      stop_velocity_threshold))
  {
    return sequence;
  }
  if (std::abs(initial_state.velocity) <= stop_velocity_threshold) {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  const bool positive_direction = initial_state.velocity > 0.0;
  const double direction = positive_direction ? 1.0 : -1.0;
  const AxisLimits stop_limits =
    directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const FeasibleInterval feasible_input =
      jerk_input_interval(state, stop_limits, time_step);
    if (!feasible_input.feasible) {
      return sequence;
    }

    const double signed_acceleration = direction * state.acceleration;
    const double signed_release_jerk =
      positive_direction ? stop_limits.native_input_max :
      -stop_limits.native_input_min;
    const double release_loss =
      jerk_release_velocity_loss(
      signed_acceleration, signed_release_jerk, time_step);
    const double speed_to_threshold =
      std::max(
      0.0, direction * state.velocity - stop_velocity_threshold);
    const bool release_braking_acceleration =
      signed_acceleration < 0.0 &&
      speed_to_threshold <=
      release_loss + time_step * time_step *
      std::max(0.0, signed_release_jerk);

    double requested_jerk =
      positive_direction ? stop_limits.native_input_min :
      stop_limits.native_input_max;
    if (release_braking_acceleration) {
      const double signed_release_input =
        std::min(
        signed_release_jerk,
        std::max(0.0, -signed_acceleration / time_step));
      requested_jerk = direction * signed_release_input;
    }
    requested_jerk =
      std::clamp(
      requested_jerk, feasible_input.lower, feasible_input.upper);
    const ProjectedAxisStep step =
      project_jerk_step(
      state, stop_limits, requested_jerk, time_step);
    if (!step.feasible) {
      return sequence;
    }
    sequence.native_inputs.push_back(step.applied_native_input);
    sequence.states.push_back(step.state);
    state = step.state;
    if (std::abs(state.velocity) <= stop_velocity_threshold) {
      mark_terminal(sequence);
      return sequence;
    }
  }
  return sequence;
}

StopSequence generate_fir_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int maximum_steps,
  const double stop_velocity_threshold)
{
  StopSequence sequence;
  if (!stop_problem_is_valid(
      initial_state, limits, time_step, maximum_steps,
      stop_velocity_threshold) ||
    coefficients.empty() ||
    history.size() + 1u != coefficients.size() ||
    std::abs(coefficients.front()) <= 1.0e-12)
  {
    return sequence;
  }
  if (std::abs(initial_state.velocity) <= stop_velocity_threshold) {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  const bool positive_direction = initial_state.velocity > 0.0;
  const AxisLimits stop_limits =
    directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  std::vector<double> current_history = history;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const int lookahead_steps =
      std::min(
      maximum_steps - step_index,
      static_cast<int>(coefficients.size()));
    const FeasibleInterval feasible_input =
      held_fir_input_interval(
      state, stop_limits, coefficients, current_history,
      time_step, lookahead_steps);
    if (!feasible_input.feasible) {
      return sequence;
    }

    const double acceleration_lower =
      std::max(
      stop_limits.acceleration_min,
      (stop_limits.velocity_min - state.velocity) / time_step);
    const double acceleration_upper =
      std::min(
      stop_limits.acceleration_max,
      (stop_limits.velocity_max - state.velocity) / time_step);
    if (acceleration_lower > acceleration_upper) {
      return sequence;
    }
    const double requested_acceleration =
      positive_direction ? acceleration_lower : acceleration_upper;
    const double free_acceleration =
      fir_acceleration(coefficients, current_history, 0.0);
    const double requested_input =
      (requested_acceleration - free_acceleration) /
      coefficients.front();
    const ProjectedFirStep step =
      project_held_fir_step(
      state, stop_limits, coefficients, current_history,
      std::clamp(
        requested_input, feasible_input.lower, feasible_input.upper),
      time_step, lookahead_steps);
    if (!step.feasible) {
      return sequence;
    }
    sequence.native_inputs.push_back(step.applied_native_input);
    sequence.states.push_back(step.state);
    state = step.state;
    current_history = step.history;
    if (std::abs(state.velocity) <= stop_velocity_threshold) {
      mark_terminal(sequence);
      return sequence;
    }
  }
  return sequence;
}

}  // namespace f_dwa_controller
