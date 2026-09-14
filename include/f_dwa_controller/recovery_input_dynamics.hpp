// Copyright 2026 YT Lab
// SPDX-License-Identifier: MIT

// Simulation research extension. Uses the same scalar model and limits as J/F.
#ifndef F_DWA_CONTROLLER__RECOVERY_INPUT_DYNAMICS_HPP_
#define F_DWA_CONTROLLER__RECOVERY_INPUT_DYNAMICS_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "f_dwa_controller/fir_input_dynamics.hpp"
#include "f_dwa_controller/native_input_dynamics.hpp"

namespace f_dwa_controller
{

struct RecoveryRollout
{
  std::vector<AxisState> states;
  double recovery_input{0.0};
  int recovery_steps{0};
  int active_input_steps{1};
  bool valid{false};
};

inline bool recovery_state_valid(const AxisState & state, const AxisLimits & limits)
{
  constexpr double eps = 1.0e-9;
  return std::isfinite(state.velocity) && std::isfinite(state.acceleration) &&
         state.velocity >= limits.velocity_min - eps &&
         state.velocity <= limits.velocity_max + eps &&
         state.acceleration >= limits.acceleration_min - eps &&
         state.acceleration <= limits.acceleration_max + eps;
}

// Project the union of the admissible integer recovery durations onto q.
// Before intersection with the first-step interval, every duration contains
// a1=0. Their union is therefore one interval, not a collection with gaps.
inline FeasibleInterval jerk_recovery_input_interval(
  const AxisState & initial, const AxisLimits & limits, double dt, int horizon)
{
  const auto first = jerk_input_interval(initial, limits, dt);
  if (!first.feasible || horizon < 2) {return {};}
  FeasibleInterval result;
  for (int m = 1; m < horizon; ++m) {
    const double duration = m * dt;
    const double velocity_gain = dt * (m + 1) / 2.0;
    const double a_lower = std::max(-limits.native_input_max * duration,
        (limits.velocity_min - initial.velocity) / velocity_gain);
    const double a_upper = std::min(-limits.native_input_min * duration,
        (limits.velocity_max - initial.velocity) / velocity_gain);
    const double lower = std::max(first.lower, (a_lower - initial.acceleration) / dt);
    const double upper = std::min(first.upper, (a_upper - initial.acceleration) / dt);
    if (lower > upper) {continue;}
    if (!result.feasible) {result = {lower, upper, true};} else {
      result.lower = std::min(result.lower, lower); result.upper = std::max(result.upper, upper);
    }
  }
  return result;
}

inline FeasibleInterval pulsed_jerk_input_interval(
  const AxisState & initial, const AxisLimits & limits, double dt,
  int horizon, int pulse_steps)
{
  auto interval = jerk_input_interval(initial, limits, dt);
  if (!interval.feasible || horizon < 1 || pulse_steps < 1) {return {};}
  for (int k = 1; k <= horizon; ++k) {
    const double m = std::min(k, pulse_steps);
    const double a_gain = m * dt;
    const double v_gain = dt * dt * (m * (m + 1.0) / 2.0 + m * (k - m));
    interval.lower = std::max({interval.lower,
          (limits.acceleration_min - initial.acceleration) / a_gain,
          (limits.velocity_min - initial.velocity - initial.acceleration * k * dt) / v_gain});
    interval.upper = std::min({interval.upper,
          (limits.acceleration_max - initial.acceleration) / a_gain,
          (limits.velocity_max - initial.velocity - initial.acceleration * k * dt) / v_gain});
    if (interval.lower > interval.upper) {return {};}
  }
  return interval;
}

// Minimum constant reverse jerk with an integer completion time <= horizon-1.
// Infeasible means this restricted continuation family has no solution.
inline RecoveryRollout minimum_jerk_recovery(
  const AxisState & initial, const AxisLimits & limits,
  double input, double dt, int horizon)
{
  RecoveryRollout result;
  if (horizon < 2 || !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(input)) {
    return result;
  }
  const auto first = project_jerk_step(initial, limits, input, dt);
  if (!first.feasible || std::abs(first.applied_native_input - input) > 1.0e-9 ||
    !recovery_state_valid(first.state, limits))
  {
    return result;
  }
  AxisState state = first.state;
  const double a = state.acceleration;
  if (std::abs(a) > 1.0e-12) {
    const double room = a > 0.0 ? limits.velocity_max - state.velocity :
      state.velocity - limits.velocity_min;
    const double bound = a > 0.0 ? -limits.native_input_min : limits.native_input_max;
    if (room < -1.0e-9 || !std::isfinite(bound) || bound <= 0.0) {
      return result;
    }
    // New acceleration is used in velocity integration: increment=a*dt*(m-1)/2.
    const int maximum = static_cast<int>(std::floor(std::min(
        static_cast<double>(horizon - 1),
        1.0 + 2.0 * std::max(0.0, room) / (std::abs(a) * dt)) + 1.0e-10));
    const double minimum = std::ceil(std::abs(a) / (bound * dt) - 1.0e-10);
    if (maximum < 1 || maximum < minimum) {
      return result;
    }
    result.recovery_steps = maximum;
    result.recovery_input = -a / (maximum * dt);
  }
  result.states.reserve(horizon);
  result.states.push_back(state);
  for (int k = 1; k < horizon; ++k) {
    const double jerk = k <= result.recovery_steps ? result.recovery_input : 0.0;
    const auto next = project_jerk_step(state, limits, jerk, dt);
    if (!next.feasible || std::abs(next.applied_native_input - jerk) > 1.0e-9 ||
      !recovery_state_valid(next.state, limits))
    {
      return RecoveryRollout{};
    }
    state = next.state;
    result.states.push_back(state);
  }
  result.valid = std::abs(state.acceleration) <= 1.0e-9;
  return result;
}

struct RecoveryFirResponse
{
  std::vector<AxisState> free_states;
  std::vector<AxisState> candidate_states;
  std::vector<AxisState> recovery_states;
  int horizon{0};
  int recovery_steps{0};
  bool valid{false};
};

// Cache these three bases once per axis and pulse, not once per candidate.
inline RecoveryFirResponse prepare_recovery_fir_response(
  const AxisState & initial, const std::vector<double> & coefficients,
  const std::vector<double> & history, double dt, int horizon,
  int candidate_steps, int recovery_steps)
{
  RecoveryFirResponse result;
  if (coefficients.empty() || history.size() + 1 != coefficients.size() ||
    candidate_steps < 1 || candidate_steps > horizon || recovery_steps < 1 ||
    !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(initial.velocity))
  {
    return result;
  }
  result.horizon = horizon;
  result.recovery_steps = recovery_steps;
  const int full_steps = std::max(horizon,
    candidate_steps + recovery_steps + static_cast<int>(coefficients.size()) - 1);
  auto memory = history;
  std::vector<double> candidate_memory(history.size(), 0.0);
  std::vector<double> recovery_memory(history.size(), 0.0);
  double v = initial.velocity, qv = 0.0, rv = 0.0;
  for (int k = 0; k < full_steps; ++k) {
    const double q = k < candidate_steps ? 1.0 : 0.0;
    const double r = k >= candidate_steps && k < candidate_steps + recovery_steps ? 1.0 : 0.0;
    const double a = fir_acceleration(coefficients, memory, 0.0);
    const double qa = fir_acceleration(coefficients, candidate_memory, q);
    const double ra = fir_acceleration(coefficients, recovery_memory, r);
    if (!std::isfinite(a) || !std::isfinite(qa) || !std::isfinite(ra)) {
      return RecoveryFirResponse{};
    }
    v += a * dt; qv += qa * dt; rv += ra * dt;
    result.free_states.push_back({v, a});
    result.candidate_states.push_back({qv, qa});
    result.recovery_states.push_back({rv, ra});
    push_fir_input(memory, 0.0);
    push_fir_input(candidate_memory, q);
    push_fir_input(recovery_memory, r);
  }
  result.valid = true;
  return result;
}

inline bool intersect_recovery_bound(
  double offset, double gain, double low, double high, double & lower, double & upper)
{
  if (!std::isfinite(offset) || !std::isfinite(gain)) {
    return false;
  }
  if (std::abs(gain) <= 1.0e-12) {
    return offset >= low - 1.0e-12 && offset <= high + 1.0e-12;
  }
  const double a = (low - offset) / gain, b = (high - offset) / gain;
  lower = std::max(lower, std::min(a, b));
  upper = std::min(upper, std::max(a, b));
  return lower <= upper;
}

// The feasible (candidate input q, recovery input r) set is convex. Clip its
// bounded rectangle with every affine state constraint, then project onto q.
// This is done once per axis; the resulting interval gets exactly N samples.
inline FeasibleInterval fir_recovery_input_interval(
  const RecoveryFirResponse & response, const AxisLimits & limits)
{
  if (!response.valid) {return {};}
  struct Point {double q; double r;};
  const double lo = limits.native_input_min, hi = limits.native_input_max;
  std::vector<Point> polygon{{lo, lo}, {hi, lo}, {hi, hi}, {lo, hi}};
  const auto clip = [&](double q_gain, double r_gain, double bound) {
      std::vector<Point> next;
      if (polygon.empty()) {return;}
      Point previous = polygon.back();
      double previous_value = q_gain * previous.q + r_gain * previous.r - bound;
      for (const auto & point : polygon) {
        const double value = q_gain * point.q + r_gain * point.r - bound;
        const bool inside = value <= 0.0, previous_inside = previous_value <= 0.0;
        if (inside != previous_inside) {
          const double fraction = previous_value / (previous_value - value);
          next.push_back({previous.q + fraction * (point.q - previous.q),
              previous.r + fraction * (point.r - previous.r)});
        }
        if (inside) {next.push_back(point);}
        previous = point; previous_value = value;
      }
      polygon = std::move(next);
    };
  for (std::size_t k = 0; k < response.free_states.size(); ++k) {
    const auto & f = response.free_states[k];
    const auto & q = response.candidate_states[k];
    const auto & r = response.recovery_states[k];
    clip(q.velocity, r.velocity, limits.velocity_max - f.velocity);
    clip(-q.velocity, -r.velocity, f.velocity - limits.velocity_min);
    clip(q.acceleration, r.acceleration, limits.acceleration_max - f.acceleration);
    clip(-q.acceleration, -r.acceleration, f.acceleration - limits.acceleration_min);
    if (polygon.empty()) {return {};}
  }
  FeasibleInterval result{polygon.front().q, polygon.front().q, true};
  for (const auto & point : polygon) {
    result.lower = std::min(result.lower, point.q);
    result.upper = std::max(result.upper, point.q);
  }
  return result;
}

inline RecoveryRollout minimum_fir_recovery(
  const RecoveryFirResponse & response, const AxisLimits & limits, double input)
{
  RecoveryRollout result;
  if (!response.valid || !std::isfinite(input) ||
    input < limits.native_input_min || input > limits.native_input_max)
  {
    return result;
  }
  double lower = limits.native_input_min, upper = limits.native_input_max;
  for (std::size_t k = 0; k < response.free_states.size(); ++k) {
    const auto & f = response.free_states[k];
    const auto & q = response.candidate_states[k];
    const auto & r = response.recovery_states[k];
    if (!intersect_recovery_bound(f.velocity + input * q.velocity, r.velocity,
        limits.velocity_min, limits.velocity_max, lower, upper) ||
      !intersect_recovery_bound(f.acceleration + input * q.acceleration, r.acceleration,
        limits.acceleration_min, limits.acceleration_max, lower, upper))
    {
      return result;
    }
  }
  result.recovery_input = std::clamp(0.0, lower, upper);
  result.recovery_steps = response.recovery_steps;
  for (std::size_t k = 0; k < response.free_states.size(); ++k) {
    const auto & f = response.free_states[k];
    const auto & q = response.candidate_states[k];
    const auto & r = response.recovery_states[k];
    const AxisState state{
      f.velocity + input * q.velocity + result.recovery_input * r.velocity,
      f.acceleration + input * q.acceleration + result.recovery_input * r.acceleration};
    if (!recovery_state_valid(state, limits)) {
      return RecoveryRollout{};
    }
    if (static_cast<int>(k) < response.horizon) {
      result.states.push_back(state);
    }
  }
  result.valid = result.states.size() == static_cast<std::size_t>(response.horizon);
  return result;
}

}  // namespace f_dwa_controller
#endif  // F_DWA_CONTROLLER__RECOVERY_INPUT_DYNAMICS_HPP_
