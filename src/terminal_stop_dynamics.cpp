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
#include <limits>

#include "f_dwa_controller/fir_input_dynamics.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr double kBoundTightening = 2.0e-7;
constexpr double kIntervalTolerance = 1.0e-12;

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

class FirHeldResponse
{
public:
  FirHeldResponse(
    const std::vector<double> & coefficients,
    const std::vector<double> & history,
    const double terminal_threshold,
    const double time_step,
    const FirStopCoefficientResponse * coefficient_response)
  : coefficients_(coefficients),
    history_ring_(history),
    terminal_threshold_(terminal_threshold)
  {
    valid_ =
      !coefficients_.empty() &&
      history.size() + 1u == coefficients_.size() &&
      std::isfinite(terminal_threshold_) &&
      terminal_threshold_ > 0.0 &&
      std::all_of(
      coefficients_.begin(), coefficients_.end(),
      [](const double value) {return std::isfinite(value);}) &&
      std::all_of(
      history.begin(), history.end(),
      [](const double value) {return std::isfinite(value);});
    if (!valid_) {
      return;
    }

    const int history_size = static_cast<int>(history.size());
    for (int history_index = 0;
      history_index < history_size; ++history_index)
    {
      if (std::abs(history[static_cast<std::size_t>(history_index)]) >
        terminal_threshold_)
      {
        history_clear_steps_ =
          std::max(history_clear_steps_, history_size - history_index);
      }
    }

    zero_input_accelerations_.reserve(coefficients_.size());
    std::vector<double> zero_input_history = history;
    for (std::size_t step_index = 0;
      step_index < coefficients_.size(); ++step_index)
    {
      zero_input_accelerations_.push_back(
        fir_acceleration(
          coefficients_, zero_input_history, 0.0));
      push_fir_input(zero_input_history, 0.0);
    }

    if (coefficient_response &&
      coefficient_response->valid &&
      coefficient_response->coefficients == coefficients_ &&
      coefficient_response->time_step == time_step &&
      coefficient_response->held_unit_accelerations.size() ==
      coefficients_.size() &&
      coefficient_response->held_unit_velocities.size() ==
      coefficients_.size())
    {
      held_unit_accelerations_ =
        &coefficient_response->held_unit_accelerations;
      held_unit_velocities_ =
        &coefficient_response->held_unit_velocities;
    } else {
      owned_coefficient_response_ =
        prepare_fir_stop_coefficient_response(coefficients_, time_step);
      if (!owned_coefficient_response_.valid) {
        valid_ = false;
        return;
      }
      held_unit_accelerations_ =
        &owned_coefficient_response_.held_unit_accelerations;
      held_unit_velocities_ =
        &owned_coefficient_response_.held_unit_velocities;
    }
  }

  [[nodiscard]] bool valid() const
  {
    return valid_;
  }

  [[nodiscard]] double zero_input_acceleration() const
  {
    if (!valid_ || zero_input_accelerations_.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return zero_input_accelerations_.front();
  }

  FeasibleInterval input_interval(
    const AxisState & state,
    const AxisLimits & limits,
    const double time_step,
    const int remaining_steps,
    bool & zero_input_reaches_terminal) const
  {
    zero_input_reaches_terminal = false;
    FeasibleInterval interval;
    interval.lower = limits.native_input_min;
    interval.upper = limits.native_input_max;
    if (!valid_ ||
      !std::isfinite(state.velocity) ||
      !std::isfinite(state.acceleration) ||
      !std::isfinite(time_step) || time_step <= 0.0 ||
      remaining_steps <= 0 ||
      limits.velocity_min > limits.velocity_max ||
      limits.acceleration_min > limits.acceleration_max ||
      limits.native_input_min > limits.native_input_max ||
      static_cast<std::size_t>(remaining_steps) >
      zero_input_accelerations_.size())
    {
      return interval;
    }

    double free_velocity = state.velocity;
    double unit_velocity = 0.0;
    bool zero_input_feasible =
      0.0 >= limits.native_input_min - kIntervalTolerance &&
      0.0 <= limits.native_input_max + kIntervalTolerance;
    for (int step_index = 0;
      step_index < remaining_steps; ++step_index)
    {
      const std::size_t index =
        static_cast<std::size_t>(step_index);
      const double free_acceleration =
        zero_input_accelerations_[index];
      const double unit_acceleration =
        (*held_unit_accelerations_)[index];
      free_velocity += time_step * free_acceleration;
      unit_velocity = (*held_unit_velocities_)[index];
      zero_input_feasible =
        zero_input_feasible &&
        free_acceleration >=
        limits.acceleration_min - kIntervalTolerance &&
        free_acceleration <=
        limits.acceleration_max + kIntervalTolerance &&
        free_velocity >= limits.velocity_min - kIntervalTolerance &&
        free_velocity <= limits.velocity_max + kIntervalTolerance;
      const int completed_steps = step_index + 1;
      if (zero_input_feasible &&
        completed_steps >= history_clear_steps_ &&
        std::abs(free_velocity) <= terminal_threshold_ &&
        std::abs(free_acceleration) <= terminal_threshold_)
      {
        zero_input_reaches_terminal = true;
        interval.lower = 0.0;
        interval.upper = 0.0;
        interval.feasible = true;
        return interval;
      }
      if (!intersect_affine_bounds(
          free_acceleration, unit_acceleration,
          limits.acceleration_min, limits.acceleration_max,
          interval.lower, interval.upper) ||
        !intersect_affine_bounds(
          free_velocity, unit_velocity,
          limits.velocity_min, limits.velocity_max,
          interval.lower, interval.upper))
      {
        return interval;
      }
    }
    interval.feasible = true;
    return interval;
  }

  bool apply_projected_step(
    AxisState & state,
    const AxisLimits & limits,
    const double applied_native_input,
    const double time_step)
  {
    if (!valid_ || zero_input_accelerations_.empty() ||
      !std::isfinite(applied_native_input) ||
      applied_native_input <
      limits.native_input_min - kIntervalTolerance ||
      applied_native_input >
      limits.native_input_max + kIntervalTolerance)
    {
      return false;
    }

    AxisState next_state;
    next_state.acceleration =
      coefficients_.front() * applied_native_input +
      zero_input_accelerations_.front();
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
    for (std::size_t index = 0;
      index + 1u < zero_input_accelerations_.size(); ++index)
    {
      zero_input_accelerations_[index] =
        coefficients_[index + 1u] * applied_native_input +
        zero_input_accelerations_[index + 1u];
    }
    zero_input_accelerations_.back() = 0.0;
    push_history(applied_native_input);
    return true;
  }

  [[nodiscard]] bool history_is_terminal() const
  {
    return history_clear_steps_ == 0;
  }

  [[nodiscard]] std::vector<double> history() const
  {
    std::vector<double> ordered_history(history_ring_.size(), 0.0);
    for (std::size_t index = 0; index < history_ring_.size(); ++index) {
      ordered_history[index] =
        history_ring_[(history_head_ + index) % history_ring_.size()];
    }
    return ordered_history;
  }

private:
  void push_history(const double input)
  {
    if (history_clear_steps_ > 0) {
      --history_clear_steps_;
    }
    if (std::abs(input) > terminal_threshold_) {
      history_clear_steps_ = static_cast<int>(history_ring_.size());
    }
    if (history_ring_.empty()) {
      return;
    }
    history_head_ =
      (history_head_ + history_ring_.size() - 1u) %
      history_ring_.size();
    history_ring_[history_head_] = input;
  }

  const std::vector<double> & coefficients_;
  std::vector<double> zero_input_accelerations_;
  FirStopCoefficientResponse owned_coefficient_response_;
  const std::vector<double> * held_unit_accelerations_{nullptr};
  const std::vector<double> * held_unit_velocities_{nullptr};
  std::vector<double> history_ring_;
  std::size_t history_head_{0u};
  double terminal_threshold_{0.0};
  int history_clear_steps_{0};
  bool valid_{false};
};

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

double jerk_release_terminal_velocity(
  const double signed_velocity,
  const double signed_acceleration,
  const double signed_current_jerk,
  const double signed_release_jerk,
  const double time_step)
{
  const double next_acceleration =
    signed_acceleration + time_step * signed_current_jerk;
  const double next_velocity =
    signed_velocity + time_step * next_acceleration;
  return next_velocity -
         jerk_release_velocity_loss(
    next_acceleration, signed_release_jerk, time_step);
}

void mark_terminal(StopSequence & sequence)
{
  sequence.feasible = true;
  sequence.terminal_state_cleared = true;
}

bool state_is_terminal(
  const AxisState & state,
  const double terminal_threshold)
{
  return std::abs(state.velocity) <= terminal_threshold &&
         std::abs(state.acceleration) <= terminal_threshold;
}

bool positive_stop_direction(const AxisState & state)
{
  return state.velocity > 0.0 ||
         (state.velocity == 0.0 && state.acceleration >= 0.0);
}

}  // namespace

FirStopCoefficientResponse prepare_fir_stop_coefficient_response(
  const std::vector<double> & coefficients,
  const double time_step)
{
  FirStopCoefficientResponse response;
  response.coefficients = coefficients;
  response.time_step = time_step;
  if (coefficients.empty() ||
    !std::isfinite(time_step) || time_step <= 0.0 ||
    !std::all_of(
      coefficients.begin(), coefficients.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return response;
  }

  response.held_unit_accelerations.reserve(coefficients.size());
  response.held_unit_velocities.reserve(coefficients.size());
  std::vector<double> held_unit_history(
    coefficients.size() - 1u, 0.0);
  double unit_velocity = 0.0;
  for (std::size_t step_index = 0u;
    step_index < coefficients.size(); ++step_index)
  {
    const double unit_acceleration =
      fir_acceleration(coefficients, held_unit_history, 1.0);
    unit_velocity += time_step * unit_acceleration;
    response.held_unit_accelerations.push_back(unit_acceleration);
    response.held_unit_velocities.push_back(unit_velocity);
    push_fir_input(held_unit_history, 1.0);
  }
  response.valid = true;
  return response;
}

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
  if (state_is_terminal(initial_state, stop_velocity_threshold)) {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  const bool positive_direction = positive_stop_direction(initial_state);
  const AxisLimits stop_limits =
    directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const bool velocity_is_captured =
      std::abs(state.velocity) <= stop_velocity_threshold;
    const double requested_acceleration =
      velocity_is_captured ? 0.0 :
      (positive_direction ? stop_limits.native_input_min :
      stop_limits.native_input_max);
    const ProjectedAxisStep step =
      project_acceleration_step(
      state, velocity_is_captured ? limits : stop_limits,
      requested_acceleration, time_step);
    if (!step.feasible) {
      return sequence;
    }
    sequence.native_inputs.push_back(step.applied_native_input);
    sequence.states.push_back(step.state);
    state = step.state;
    if (state_is_terminal(state, stop_velocity_threshold)) {
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
  if (state_is_terminal(initial_state, stop_velocity_threshold)) {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  const bool positive_direction = positive_stop_direction(initial_state);
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

    const double signed_velocity = direction * state.velocity;
    const double signed_acceleration = direction * state.acceleration;
    const double signed_input_lower =
      positive_direction ? feasible_input.lower : -feasible_input.upper;
    const double signed_input_upper =
      positive_direction ? feasible_input.upper : -feasible_input.lower;
    double requested_signed_jerk = signed_input_lower;
    if (signed_acceleration < 0.0 && signed_input_upper > 0.0) {
      const double signed_release_input =
        std::min(
        signed_input_upper,
        std::max(0.0, -signed_acceleration / time_step));
      const double release_terminal_velocity =
        jerk_release_terminal_velocity(
        signed_velocity, signed_acceleration, signed_release_input,
        signed_input_upper, time_step);
      if (release_terminal_velocity < -kIntervalTolerance) {
        return sequence;
      }
      if (release_terminal_velocity <= stop_velocity_threshold) {
        requested_signed_jerk = signed_release_input;
      } else {
        const double braking_terminal_velocity =
          jerk_release_terminal_velocity(
          signed_velocity, signed_acceleration, signed_input_lower,
          signed_input_upper, time_step);
        if (braking_terminal_velocity <= stop_velocity_threshold) {
          const double target_terminal_velocity =
            0.5 * stop_velocity_threshold;
          if (braking_terminal_velocity >= target_terminal_velocity) {
            requested_signed_jerk = signed_input_lower;
          } else {
            double lower_jerk = signed_input_lower;
            double upper_jerk = signed_release_input;
            for (int search_index = 0; search_index < 32; ++search_index) {
              const double midpoint_jerk =
                0.5 * (lower_jerk + upper_jerk);
              const double midpoint_terminal_velocity =
                jerk_release_terminal_velocity(
                signed_velocity, signed_acceleration, midpoint_jerk,
                signed_input_upper, time_step);
              if (midpoint_terminal_velocity <
                target_terminal_velocity)
              {
                lower_jerk = midpoint_jerk;
              } else {
                upper_jerk = midpoint_jerk;
              }
            }
            requested_signed_jerk =
              0.5 * (lower_jerk + upper_jerk);
          }
        }
      }
    }
    const double requested_jerk =
      std::clamp(
      direction * requested_signed_jerk,
      feasible_input.lower, feasible_input.upper);
    const ProjectedAxisStep step =
      project_jerk_step(
      state, stop_limits, requested_jerk, time_step);
    if (!step.feasible) {
      return sequence;
    }
    sequence.native_inputs.push_back(step.applied_native_input);
    sequence.states.push_back(step.state);
    state = step.state;
    if (state_is_terminal(state, stop_velocity_threshold)) {
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
  const double stop_velocity_threshold,
  const bool record_fir_histories,
  const FirStopCoefficientResponse * coefficient_response)
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
  FirHeldResponse held_response(
    coefficients, history, stop_velocity_threshold, time_step,
    coefficient_response);
  if (!held_response.valid()) {
    return sequence;
  }
  if (state_is_terminal(initial_state, stop_velocity_threshold) &&
    held_response.history_is_terminal())
  {
    sequence.feasible = true;
    sequence.terminal_state_cleared = true;
    return sequence;
  }

  AxisState direction_state = initial_state;
  if (direction_state.velocity == 0.0 &&
    direction_state.acceleration == 0.0)
  {
    direction_state.acceleration =
      held_response.zero_input_acceleration();
  }
  const bool positive_direction = positive_stop_direction(direction_state);
  const AxisLimits stop_limits =
    directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  bool zero_input_tail_active = false;
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const int lookahead_steps =
      std::min(
      maximum_steps - step_index,
      static_cast<int>(coefficients.size()));
    bool zero_input_reaches_terminal = false;
    const FeasibleInterval feasible_input =
      held_response.input_interval(
      state, stop_limits, time_step, lookahead_steps,
      zero_input_reaches_terminal);
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
    if (zero_input_reaches_terminal) {
      zero_input_tail_active = true;
    }
    double requested_input = 0.0;
    if (!zero_input_tail_active) {
      const double requested_acceleration =
        positive_direction ? acceleration_lower : acceleration_upper;
      const double free_acceleration =
        held_response.zero_input_acceleration();
      requested_input =
        (requested_acceleration - free_acceleration) /
        coefficients.front();
    }
    const double applied_native_input =
      std::clamp(
      requested_input, feasible_input.lower, feasible_input.upper);
    if (!held_response.apply_projected_step(
        state, stop_limits,
        applied_native_input, time_step))
    {
      return sequence;
    }
    sequence.native_inputs.push_back(applied_native_input);
    sequence.states.push_back(state);
    if (record_fir_histories) {
      sequence.fir_histories.push_back(held_response.history());
    }
    if (state_is_terminal(state, stop_velocity_threshold) &&
      held_response.history_is_terminal())
    {
      mark_terminal(sequence);
      return sequence;
    }
  }
  return sequence;
}

}  // namespace f_dwa_controller
