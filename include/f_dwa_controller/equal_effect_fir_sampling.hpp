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

// Experimental F-DWA sampling: coarse effect-gap refinement with a maximum
// feasible q,...,q,beta*q,0,... pulse. The default trajectory generator does
// not use this path unless its dedicated runtime parameter is enabled.
#ifndef F_DWA_CONTROLLER__EQUAL_EFFECT_FIR_SAMPLING_HPP_
#define F_DWA_CONTROLLER__EQUAL_EFFECT_FIR_SAMPLING_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "f_dwa_controller/fir_input_dynamics.hpp"
#include "f_dwa_controller/recovery_input_dynamics.hpp"

namespace f_dwa_controller
{

struct EqualEffectFirPulse
{
  std::vector<AxisState> states;
  double native_input{0.0};
  double first_native_input{0.0};
  double horizon_effect{0.0};
  double equivalent_duration{0.0};
  int full_input_steps{0};
  double fractional_last_input{0.0};
  bool zero_input_duration_unbounded{false};
  bool valid{false};
};

struct EqualEffectFirSamples
{
  std::vector<EqualEffectFirPulse> pulses;
  std::vector<int> interval_refinement_counts;
  int coarse_evaluation_count{0};
  int refinement_evaluation_count{0};
  std::string failure_reason;
  bool converged{false};
};

struct EqualEffectFirContext
{
  AxisState initial;
  AxisLimits limits;
  std::vector<double> coefficients;
  std::vector<double> step_response;
  std::vector<double> integrated_step_response;
  std::vector<double> free_acceleration;
  std::vector<double> free_velocity;
  double coefficient_sum{0.0};
  double free_terminal_velocity{0.0};
  double dt{0.0};
  int horizon{0};
  bool valid{false};
};

inline double equal_effect_step_at(const EqualEffectFirContext & context, const int index)
{
  if (index < 0) {return 0.0;}
  if (index < static_cast<int>(context.step_response.size())) {
    return context.step_response[static_cast<std::size_t>(index)];
  }
  return context.coefficient_sum;
}

inline double equal_effect_integral_at(
  const EqualEffectFirContext & context, const int index)
{
  if (index < 0) {return 0.0;}
  const int size = static_cast<int>(context.integrated_step_response.size());
  if (index < size) {
    return context.integrated_step_response[static_cast<std::size_t>(index)];
  }
  return context.integrated_step_response.back() +
         (index - size + 1) * context.coefficient_sum;
}

inline double equal_effect_coefficient_at(
  const EqualEffectFirContext & context, const int index)
{
  return index >= 0 && index < static_cast<int>(context.coefficients.size()) ?
         context.coefficients[static_cast<std::size_t>(index)] : 0.0;
}

inline double equal_effect_free_acceleration_at(
  const EqualEffectFirContext & context, const int index)
{
  return index >= 0 && index < static_cast<int>(context.free_acceleration.size()) ?
         context.free_acceleration[static_cast<std::size_t>(index)] : 0.0;
}

inline double equal_effect_free_velocity_at(
  const EqualEffectFirContext & context, const int index)
{
  return index >= 0 && index < static_cast<int>(context.free_velocity.size()) ?
         context.free_velocity[static_cast<std::size_t>(index)] :
         context.free_terminal_velocity;
}

inline EqualEffectFirContext prepare_equal_effect_fir_context(
  const AxisState & initial, const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & initial_history,
  const double dt, const int horizon)
{
  EqualEffectFirContext context;
  if (coefficients.empty() || initial_history.size() + 1u != coefficients.size() ||
    !std::isfinite(initial.velocity) || !std::isfinite(initial.acceleration) ||
    !std::isfinite(dt) || dt <= 0.0 || horizon < 1 ||
    limits.native_input_min > 0.0 || limits.native_input_max < 0.0)
  {
    return context;
  }
  context.initial = initial;
  context.limits = limits;
  context.coefficients = coefficients;
  context.dt = dt;
  context.horizon = horizon;
  context.step_response.reserve(coefficients.size());
  context.integrated_step_response.reserve(coefficients.size());
  double step = 0.0;
  double integrated = 0.0;
  for (const double coefficient : coefficients) {
    if (!std::isfinite(coefficient)) {return EqualEffectFirContext{};}
    step += coefficient;
    integrated += step;
    context.step_response.push_back(step);
    context.integrated_step_response.push_back(integrated);
  }
  context.coefficient_sum = step;
  if (!std::isfinite(step) || std::abs(step) <= 1.0e-12) {
    return EqualEffectFirContext{};
  }

  auto history = initial_history;
  double velocity = initial.velocity;
  context.free_acceleration.reserve(coefficients.size());
  context.free_velocity.reserve(coefficients.size());
  for (std::size_t index = 0u; index < coefficients.size(); ++index) {
    const double acceleration = fir_acceleration(coefficients, history, 0.0);
    if (!std::isfinite(acceleration)) {return EqualEffectFirContext{};}
    velocity += dt * acceleration;
    context.free_acceleration.push_back(acceleration);
    context.free_velocity.push_back(velocity);
    push_fir_input(history, 0.0);
  }
  context.free_terminal_velocity = velocity;
  context.valid = true;
  return context;
}

inline bool intersect_equal_effect_beta_bound(
  const double offset, const double gain, const double lower_bound,
  const double upper_bound, double & beta_lower, double & beta_upper)
{
  if (!std::isfinite(offset) || !std::isfinite(gain)) {return false;}
  if (std::abs(gain) <= 1.0e-14) {
    return offset >= lower_bound - 1.0e-10 &&
           offset <= upper_bound + 1.0e-10;
  }
  const double first = (lower_bound - offset) / gain;
  const double second = (upper_bound - offset) / gain;
  beta_lower = std::max(beta_lower, std::min(first, second));
  beta_upper = std::min(beta_upper, std::max(first, second));
  return beta_lower <= beta_upper + 1.0e-10;
}

inline bool equal_effect_beta_interval(
  const EqualEffectFirContext & context, const double input,
  const int full_input_steps, double & beta_lower, double & beta_upper)
{
  if (!context.valid || full_input_steps < 0) {return false;}
  beta_lower = 0.0;
  beta_upper = 1.0;
  const int full_count = std::max(
    context.horizon,
    full_input_steps + static_cast<int>(context.coefficients.size()));
  for (int index = 0; index < full_count; ++index) {
    const double acceleration_offset =
      equal_effect_free_acceleration_at(context, index) + input *
      (equal_effect_step_at(context, index) -
      equal_effect_step_at(context, index - full_input_steps));
    const double acceleration_gain =
      input * equal_effect_coefficient_at(context, index - full_input_steps);
    if (!intersect_equal_effect_beta_bound(
        acceleration_offset, acceleration_gain,
        context.limits.acceleration_min, context.limits.acceleration_max,
        beta_lower, beta_upper))
    {
      return false;
    }

    const double velocity_offset =
      equal_effect_free_velocity_at(context, index) + context.dt * input *
      (equal_effect_integral_at(context, index) -
      equal_effect_integral_at(context, index - full_input_steps));
    const double velocity_gain = context.dt * input *
      equal_effect_step_at(context, index - full_input_steps);
    if (!intersect_equal_effect_beta_bound(
        velocity_offset, velocity_gain,
        context.limits.velocity_min, context.limits.velocity_max,
        beta_lower, beta_upper))
    {
      return false;
    }
  }
  beta_lower = std::max(0.0, beta_lower);
  beta_upper = std::min(1.0, beta_upper);
  return beta_lower <= beta_upper + 1.0e-10;
}

inline EqualEffectFirPulse make_equal_effect_fir_pulse(
  const EqualEffectFirContext & context, const double input,
  const int full_input_steps, const double beta,
  const bool zero_input_duration_unbounded)
{
  EqualEffectFirPulse pulse;
  if (!context.valid || full_input_steps < 0 || !std::isfinite(beta) ||
    beta < -1.0e-10 || beta > 1.0 + 1.0e-10)
  {
    return pulse;
  }
  pulse.native_input = input;
  pulse.full_input_steps = full_input_steps;
  pulse.fractional_last_input = std::clamp(beta, 0.0, 1.0);
  pulse.equivalent_duration =
    (full_input_steps + pulse.fractional_last_input) * context.dt;
  pulse.zero_input_duration_unbounded = zero_input_duration_unbounded;
  pulse.first_native_input = full_input_steps >= 1 ? input :
    pulse.fractional_last_input * input;
  pulse.states.reserve(static_cast<std::size_t>(context.horizon));
  for (int index = 0; index < context.horizon; ++index) {
    const double acceleration =
      equal_effect_free_acceleration_at(context, index) + input *
      (equal_effect_step_at(context, index) -
      equal_effect_step_at(context, index - full_input_steps)) +
      pulse.fractional_last_input * input *
      equal_effect_coefficient_at(context, index - full_input_steps);
    const double velocity =
      equal_effect_free_velocity_at(context, index) + context.dt * input *
      (equal_effect_integral_at(context, index) -
      equal_effect_integral_at(context, index - full_input_steps)) +
      context.dt * pulse.fractional_last_input * input *
      equal_effect_step_at(context, index - full_input_steps);
    const AxisState state{velocity, acceleration};
    if (!recovery_state_valid(state, context.limits)) {
      return EqualEffectFirPulse{};
    }
    pulse.states.push_back(state);
    pulse.horizon_effect += context.dt * velocity;
  }
  pulse.valid = true;
  return pulse;
}

inline EqualEffectFirPulse maximum_feasible_equal_effect_fir_pulse(
  const EqualEffectFirContext & context, const double input)
{
  EqualEffectFirPulse pulse;
  if (!context.valid || !std::isfinite(input) ||
    input < context.limits.native_input_min - 1.0e-12 ||
    input > context.limits.native_input_max + 1.0e-12)
  {
    return pulse;
  }
  if (std::abs(input) <= 1.0e-14) {
    double lower = 0.0;
    double upper = 1.0;
    if (!equal_effect_beta_interval(context, 0.0, 0, lower, upper)) {
      return pulse;
    }
    return make_equal_effect_fir_pulse(context, 0.0, 0, 0.0, true);
  }

  const double final_velocity_rate =
    context.dt * input * context.coefficient_sum;
  if (!std::isfinite(final_velocity_rate) ||
    std::abs(final_velocity_rate) <= 1.0e-14)
  {
    return pulse;
  }
  const double limiting_velocity = final_velocity_rate > 0.0 ?
    context.limits.velocity_max : context.limits.velocity_min;
  const double maximum_equivalent_steps =
    (limiting_velocity - context.free_terminal_velocity) /
    final_velocity_rate;
  if (!std::isfinite(maximum_equivalent_steps) ||
    maximum_equivalent_steps < -1.0e-10)
  {
    return pulse;
  }
  constexpr int kMaximumEnumeratedDurationSteps = 4096;
  const double guarded_steps = std::floor(
    std::max(0.0, maximum_equivalent_steps) + 1.0e-10) + 1.0;
  if (guarded_steps > kMaximumEnumeratedDurationSteps) {return pulse;}
  const int maximum_steps = static_cast<int>(guarded_steps);
  for (int full_input_steps = maximum_steps;
    full_input_steps >= 0; --full_input_steps)
  {
    double beta_lower = 0.0;
    double beta_upper = 1.0;
    if (!equal_effect_beta_interval(
        context, input, full_input_steps, beta_lower, beta_upper))
    {
      continue;
    }
    const double beta = std::clamp(beta_upper, 0.0, 1.0);
    if (beta < beta_lower - 1.0e-10) {continue;}
    pulse = make_equal_effect_fir_pulse(
      context, input, full_input_steps, beta, false);
    if (pulse.valid) {return pulse;}
  }
  return EqualEffectFirPulse{};
}

inline std::vector<int> allocate_equal_effect_refinements(
  const std::vector<double> & effect_gaps, const int refinement_count)
{
  std::vector<int> allocations(effect_gaps.size(), 0);
  if (effect_gaps.empty() || refinement_count < 0) {return {};}
  for (int addition = 0; addition < refinement_count; ++addition) {
    std::size_t selected = 0u;
    double selected_gap = -1.0;
    for (std::size_t index = 0u; index < effect_gaps.size(); ++index) {
      const double predicted_gap = effect_gaps[index] /
        static_cast<double>(allocations[index] + 1);
      if (!std::isfinite(predicted_gap) || predicted_gap < 0.0) {return {};}
      // Do not concentrate equal-score additions in an earlier interval only
      // because it is visited first.
      if (predicted_gap > selected_gap ||
        (predicted_gap == selected_gap &&
        allocations[index] < allocations[selected]))
      {
        selected = index;
        selected_gap = predicted_gap;
      }
    }
    ++allocations[selected];
  }
  return allocations;
}

inline EqualEffectFirSamples sample_equal_effect_fir_axis(
  const EqualEffectFirContext & context, const int sample_count)
{
  EqualEffectFirSamples result;
  if (!context.valid || sample_count < 3 || sample_count % 2 == 0 ||
    !std::isfinite(context.limits.native_input_min) ||
    !std::isfinite(context.limits.native_input_max) ||
    context.limits.native_input_min >= context.limits.native_input_max)
  {
    result.failure_reason = "invalid context, input range, or non-odd sample count";
    return result;
  }

  const int coarse_count = (sample_count + 1) / 2;
  const int refinement_count = sample_count - coarse_count;
  std::vector<EqualEffectFirPulse> coarse_pulses;
  coarse_pulses.reserve(static_cast<std::size_t>(coarse_count));
  for (int index = 0; index < coarse_count; ++index) {
    const double fraction = static_cast<double>(index) /
      static_cast<double>(coarse_count - 1);
    const double input = context.limits.native_input_min + fraction *
      (context.limits.native_input_max - context.limits.native_input_min);
    const EqualEffectFirPulse pulse =
      maximum_feasible_equal_effect_fir_pulse(context, input);
    ++result.coarse_evaluation_count;
    if (!pulse.valid) {
      result.failure_reason = "a coarse maximum-duration sample was infeasible";
      return result;
    }
    coarse_pulses.push_back(pulse);
  }

  std::vector<double> effect_gaps;
  effect_gaps.reserve(static_cast<std::size_t>(coarse_count - 1));
  for (int index = 0; index + 1 < coarse_count; ++index) {
    effect_gaps.push_back(std::abs(
        coarse_pulses[static_cast<std::size_t>(index + 1)].horizon_effect -
        coarse_pulses[static_cast<std::size_t>(index)].horizon_effect));
  }
  result.interval_refinement_counts = allocate_equal_effect_refinements(
    effect_gaps, refinement_count);
  if (result.interval_refinement_counts.size() != effect_gaps.size()) {
    result.failure_reason = "a coarse effect gap was invalid";
    return result;
  }

  result.pulses.reserve(static_cast<std::size_t>(sample_count));
  for (int index = 0; index + 1 < coarse_count; ++index) {
    const auto & lower = coarse_pulses[static_cast<std::size_t>(index)];
    const auto & upper = coarse_pulses[static_cast<std::size_t>(index + 1)];
    result.pulses.push_back(lower);
    const int additions =
      result.interval_refinement_counts[static_cast<std::size_t>(index)];
    for (int addition = 1; addition <= additions; ++addition) {
      const double fraction = static_cast<double>(addition) /
        static_cast<double>(additions + 1);
      const double input = lower.native_input + fraction *
        (upper.native_input - lower.native_input);
      const EqualEffectFirPulse pulse =
        maximum_feasible_equal_effect_fir_pulse(context, input);
      ++result.refinement_evaluation_count;
      if (!pulse.valid) {
        result.pulses.clear();
        result.failure_reason = "a refined maximum-duration sample was infeasible";
        return result;
      }
      result.pulses.push_back(pulse);
    }
  }
  result.pulses.push_back(coarse_pulses.back());
  result.converged = true;
  return result;
}

}  // namespace f_dwa_controller
#endif  // F_DWA_CONTROLLER__EQUAL_EFFECT_FIR_SAMPLING_HPP_
