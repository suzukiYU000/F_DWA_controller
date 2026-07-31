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

#include "f_dwa_controller/fir_input_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace f_dwa_controller
{

namespace
{

constexpr double kIntervalTolerance = 1.0e-12;

bool valid_fir_problem(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int remaining_steps)
{
  if (!std::isfinite(state.velocity) ||
    !std::isfinite(state.acceleration) ||
    !std::isfinite(time_step) || time_step <= 0.0 ||
    remaining_steps <= 0 || coefficients.empty() ||
    history.size() + 1u != coefficients.size() ||
    limits.velocity_min > limits.velocity_max ||
    limits.acceleration_min > limits.acceleration_max ||
    limits.native_input_min > limits.native_input_max)
  {
    return false;
  }
  return std::all_of(
    coefficients.begin(), coefficients.end(),
    [](const double value) {return std::isfinite(value);}) &&
         std::all_of(
    history.begin(), history.end(),
    [](const double value) {return std::isfinite(value);});
}

bool intersect_affine_bounds(
  const double offset,
  const double gain,
  const double output_minimum,
  const double output_maximum,
  double & input_minimum,
  double & input_maximum)
{
  if (std::abs(gain) <= kIntervalTolerance) {
    return offset >= output_minimum - kIntervalTolerance &&
           offset <= output_maximum + kIntervalTolerance;
  }
  const double first = (output_minimum - offset) / gain;
  const double second = (output_maximum - offset) / gain;
  input_minimum = std::max(input_minimum, std::min(first, second));
  input_maximum = std::min(input_maximum, std::max(first, second));
  if (input_minimum > input_maximum + kIntervalTolerance) {
    return false;
  }
  if (input_minimum > input_maximum) {
    const double midpoint = 0.5 * (input_minimum + input_maximum);
    input_minimum = midpoint;
    input_maximum = midpoint;
  }
  return true;
}

}  // namespace

double fir_acceleration(
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double native_input)
{
  if (coefficients.empty() ||
    history.size() + 1u != coefficients.size() ||
    !std::isfinite(native_input))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double acceleration = coefficients.front() * native_input;
  for (std::size_t index = 0; index < history.size(); ++index) {
    acceleration += coefficients[index + 1u] * history[index];
  }
  return acceleration;
}

void push_fir_input(
  std::vector<double> & history,
  const double native_input)
{
  if (history.empty()) {
    return;
  }
  if (history.size() > 1u) {
    std::copy_backward(
      history.begin(), history.end() - 1, history.end());
  }
  history.front() = native_input;
}

FeasibleInterval held_fir_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int remaining_steps)
{
  return prepare_held_fir_affine_response(
    state, limits, coefficients, history, time_step,
    remaining_steps).input_interval;
}

HeldFirAffineResponse prepare_held_fir_affine_response(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int remaining_steps)
{
  return prepare_pulsed_fir_affine_response(
    state, limits, coefficients, history, time_step, remaining_steps,
    remaining_steps);
}

HeldFirAffineResponse prepare_pulsed_fir_affine_response(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int remaining_steps,
  const int active_input_steps)
{
  HeldFirAffineResponse response;
  response.input_interval.lower = limits.native_input_min;
  response.input_interval.upper = limits.native_input_max;
  if (!valid_fir_problem(
      state, limits, coefficients, history, time_step,
      remaining_steps) ||
    active_input_steps <= 0 ||
    active_input_steps > remaining_steps)
  {
    return response;
  }

  response.free_states.reserve(static_cast<std::size_t>(remaining_steps));
  response.unit_states.reserve(static_cast<std::size_t>(remaining_steps));
  std::vector<double> free_history = history;
  std::vector<double> unit_history(history.size(), 0.0);
  double free_velocity = state.velocity;
  double unit_velocity = 0.0;
  for (int step_index = 0;
    step_index < remaining_steps; ++step_index)
  {
    const double free_acceleration =
      fir_acceleration(coefficients, free_history, 0.0);
    const double unit_native_input =
      step_index < active_input_steps ? 1.0 : 0.0;
    const double unit_acceleration =
      fir_acceleration(
      coefficients, unit_history, unit_native_input);
    free_velocity += time_step * free_acceleration;
    unit_velocity += time_step * unit_acceleration;
    response.free_states.push_back(
      AxisState{free_velocity, free_acceleration});
    response.unit_states.push_back(
      AxisState{unit_velocity, unit_acceleration});
    if (!intersect_affine_bounds(
        free_acceleration, unit_acceleration,
        limits.acceleration_min, limits.acceleration_max,
        response.input_interval.lower, response.input_interval.upper) ||
      !intersect_affine_bounds(
        free_velocity, unit_velocity,
        limits.velocity_min, limits.velocity_max,
        response.input_interval.lower, response.input_interval.upper))
    {
      return response;
    }
    push_fir_input(free_history, 0.0);
    push_fir_input(unit_history, unit_native_input);
  }
  response.input_interval.feasible = true;
  return response;
}

bool sample_held_fir_affine_response(
  const HeldFirAffineResponse & response,
  const AxisLimits & limits,
  const double held_native_input,
  std::vector<AxisState> & states)
{
  states.clear();
  if (!response.input_interval.feasible ||
    response.free_states.size() != response.unit_states.size() ||
    !std::isfinite(held_native_input) ||
    held_native_input <
    response.input_interval.lower - kIntervalTolerance ||
    held_native_input >
    response.input_interval.upper + kIntervalTolerance ||
    held_native_input < limits.native_input_min - kIntervalTolerance ||
    held_native_input > limits.native_input_max + kIntervalTolerance)
  {
    return false;
  }

  states.reserve(response.free_states.size());
  for (std::size_t step_index = 0;
    step_index < response.free_states.size(); ++step_index)
  {
    const AxisState & free_state = response.free_states[step_index];
    const AxisState & unit_state = response.unit_states[step_index];
    AxisState sampled_state;
    sampled_state.acceleration =
      free_state.acceleration +
      held_native_input * unit_state.acceleration;
    sampled_state.velocity =
      free_state.velocity +
      held_native_input * unit_state.velocity;
    if (!std::isfinite(sampled_state.acceleration) ||
      !std::isfinite(sampled_state.velocity) ||
      sampled_state.acceleration <
      limits.acceleration_min - kIntervalTolerance ||
      sampled_state.acceleration >
      limits.acceleration_max + kIntervalTolerance ||
      sampled_state.velocity < limits.velocity_min - kIntervalTolerance ||
      sampled_state.velocity > limits.velocity_max + kIntervalTolerance)
    {
      states.clear();
      return false;
    }
    states.push_back(sampled_state);
  }
  return true;
}

ProjectedFirStep project_held_fir_step(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double native_input_reference,
  const double time_step,
  const int remaining_steps)
{
  const FeasibleInterval interval =
    held_fir_input_interval(
    state, limits, coefficients, history, time_step,
    remaining_steps);
  if (!interval.feasible || !std::isfinite(native_input_reference)) {
    return {};
  }

  ProjectedFirStep result;
  result.applied_native_input =
    std::clamp(native_input_reference, interval.lower, interval.upper);
  result.state.acceleration =
    fir_acceleration(
    coefficients, history, result.applied_native_input);
  result.state.velocity =
    state.velocity + time_step * result.state.acceleration;
  result.history = history;
  push_fir_input(result.history, result.applied_native_input);
  result.feasible =
    std::isfinite(result.state.acceleration) &&
    std::isfinite(result.state.velocity);
  return result;
}

ProjectedFirStep apply_projected_fir_step(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double projected_native_input,
  const double time_step)
{
  ProjectedFirStep result;
  if (!valid_fir_problem(
      state, limits, coefficients, history, time_step, 1) ||
    !std::isfinite(projected_native_input) ||
    projected_native_input <
    limits.native_input_min - kIntervalTolerance ||
    projected_native_input >
    limits.native_input_max + kIntervalTolerance)
  {
    return result;
  }

  result.applied_native_input = projected_native_input;
  result.state.acceleration =
    fir_acceleration(coefficients, history, projected_native_input);
  result.state.velocity =
    state.velocity + time_step * result.state.acceleration;
  result.history = history;
  push_fir_input(result.history, projected_native_input);
  result.feasible =
    std::isfinite(result.state.acceleration) &&
    std::isfinite(result.state.velocity) &&
    result.state.acceleration >=
    limits.acceleration_min - kIntervalTolerance &&
    result.state.acceleration <=
    limits.acceleration_max + kIntervalTolerance &&
    result.state.velocity >= limits.velocity_min - kIntervalTolerance &&
    result.state.velocity <= limits.velocity_max + kIntervalTolerance;
  return result;
}

bool apply_projected_fir_step_in_place(
  AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  std::vector<double> & history,
  const double projected_native_input,
  const double time_step)
{
  if (!valid_fir_problem(
      state, limits, coefficients, history, time_step, 1) ||
    !std::isfinite(projected_native_input) ||
    projected_native_input <
    limits.native_input_min - kIntervalTolerance ||
    projected_native_input >
    limits.native_input_max + kIntervalTolerance)
  {
    return false;
  }

  AxisState next_state;
  next_state.acceleration =
    fir_acceleration(coefficients, history, projected_native_input);
  next_state.velocity =
    state.velocity + time_step * next_state.acceleration;
  const bool feasible =
    std::isfinite(next_state.acceleration) &&
    std::isfinite(next_state.velocity) &&
    next_state.acceleration >=
    limits.acceleration_min - kIntervalTolerance &&
    next_state.acceleration <=
    limits.acceleration_max + kIntervalTolerance &&
    next_state.velocity >= limits.velocity_min - kIntervalTolerance &&
    next_state.velocity <= limits.velocity_max + kIntervalTolerance;
  if (!feasible) {
    return false;
  }

  state = next_state;
  push_fir_input(history, projected_native_input);
  return true;
}

}  // namespace f_dwa_controller
