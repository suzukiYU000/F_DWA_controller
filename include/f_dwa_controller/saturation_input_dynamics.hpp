#ifndef F_DWA_CONTROLLER__SATURATION_INPUT_DYNAMICS_HPP_
#define F_DWA_CONTROLLER__SATURATION_INPUT_DYNAMICS_HPP_

#include "f_dwa_controller/recovery_input_dynamics.hpp"

namespace f_dwa_controller
{

// Apply the sampled acceleration for the longest admissible whole number of
// ticks, then coast. Acceleration is the native input, so this transition has
// no jerk constraint and never clips a sampled input or a velocity state.
inline RecoveryRollout longest_acceleration_input(
  const AxisState & initial, const AxisLimits & limits,
  double input, double dt, int horizon)
{
  RecoveryRollout result;
  const auto interval = acceleration_input_interval(initial, limits, dt);
  if (!interval.feasible || horizon < 1 || !std::isfinite(input) ||
    input < interval.lower - 1.0e-12 || input > interval.upper + 1.0e-12 ||
    limits.acceleration_min > 0.0 || limits.acceleration_max < 0.0 ||
    limits.native_input_min > 0.0 || limits.native_input_max < 0.0)
  {
    return result;
  }
  result.states.reserve(horizon);
  AxisState state = initial;
  for (int step = 0; step < horizon; ++step) {
    const AxisState next{state.velocity + input * dt, input};
    if (!recovery_state_valid(next, limits)) {break;}
    result.states.push_back(next);
    state = next;
  }
  if (result.states.empty()) {return result;}
  result.active_input_steps = static_cast<int>(result.states.size());
  state.acceleration = 0.0;
  result.states.resize(horizon, state);
  result.valid = true;
  return result;
}

inline bool constrain_input(
  FeasibleInterval & interval, double offset, double gain, double low, double high)
{
  interval.feasible = interval.feasible &&
    intersect_recovery_bound(offset, gain, low, high, interval.lower, interval.upper);
  return interval.feasible;
}

inline std::vector<FeasibleInterval> merge_input_intervals(std::vector<FeasibleInterval> intervals)
{
  intervals.erase(std::remove_if(intervals.begin(), intervals.end(),
    [](const auto & interval) {return !interval.feasible;}), intervals.end());
  std::sort(intervals.begin(), intervals.end(),
    [](const auto & first, const auto & second) {return first.lower < second.lower;});
  std::vector<FeasibleInterval> merged;
  for (const auto & interval : intervals) {
    if (merged.empty() || interval.lower > merged.back().upper) {
      merged.push_back(interval);
    } else {
      merged.back().upper = std::max(merged.back().upper, interval.upper);
    }
  }
  return merged;
}

// Use the same amplitude spacing on a connected domain; do not fill infeasible gaps.
inline std::vector<double> sample_input_intervals(
  const std::vector<FeasibleInterval> & intervals, int count)
{
  if (intervals.empty() || count < 1) {return {};}
  if (intervals.size() == 1) {return uniform_samples(intervals.front(), count);}
  double width = 0.;
  for (const auto & interval : intervals) {width += interval.upper - interval.lower;}
  std::vector<double> samples;
  for (int i = 0; i < count; ++i) {
    if (i == 0) {samples.push_back(intervals.front().lower); continue;}
    if (i == count - 1) {samples.push_back(intervals.back().upper); continue;}
    if (width == 0.) {
      samples.push_back(intervals[i * (intervals.size() - 1) / (count - 1)].lower);
      continue;
    }
    double position = width * i / (count - 1);
    for (const auto & interval : intervals) {
      const double span = interval.upper - interval.lower;
      if (position <= span) {samples.push_back(interval.lower + position); break;}
      position -= span;
    }
  }
  return samples;
}

// For fixed hold p and recovery m, every state constraint is affine in q.
inline std::vector<FeasibleInterval> jerk_switch_input_intervals(
  const AxisState & initial, const AxisLimits & limits, double dt, int horizon)
{
  if (!std::isfinite(dt) || dt <= 0. || horizon < 2 ||
    !std::isfinite(initial.velocity) || !std::isfinite(initial.acceleration)) {return {};}
  FeasibleInterval prefix{limits.native_input_min, limits.native_input_max, true};
  std::vector<FeasibleInterval> intervals;
  for (int p = 1; p <= horizon; ++p) {
    const double ag = p * dt, vg = .5 * dt * dt * p * (p + 1);
    if (!constrain_input(prefix, initial.acceleration, ag,
        limits.acceleration_min, limits.acceleration_max) ||
      !constrain_input(prefix, initial.velocity + initial.acceleration * p * dt, vg,
        limits.velocity_min, limits.velocity_max)) {break;}
    for (int m = 1; m <= horizon - p; ++m) {
      auto interval = prefix;
      constrain_input(interval, initial.acceleration, ag,
        -limits.native_input_max * m * dt, -limits.native_input_min * m * dt);
      constrain_input(interval,
        initial.velocity + initial.acceleration * dt * (p + .5 * (m - 1)),
        .5 * dt * dt * p * (p + m), limits.velocity_min, limits.velocity_max);
      if (interval.feasible) {intervals.push_back(interval);}
    }
    if (p == horizon) {
      auto zero_acceleration = prefix;
      constrain_input(zero_acceleration, initial.acceleration, ag, 0., 0.);
      if (zero_acceleration.feasible) {intervals.push_back(zero_acceleration);}
    }
  }
  return merge_input_intervals(std::move(intervals));
}

inline RecoveryRollout longest_jerk_recovery(
  const AxisState & initial, const AxisLimits & limits,
  double input, double dt, int horizon)
{
  RecoveryRollout result;
  if (!std::isfinite(input) || input < limits.native_input_min - 1e-12 ||
    input > limits.native_input_max + 1e-12 || !std::isfinite(dt) || dt <= 0. || horizon < 2) {
    return result;
  }
  AxisState state = initial;
  std::vector<AxisState> prefix;
  int selected = 0;
  for (int p = 1; p <= horizon; ++p) {
    state.acceleration += input * dt;
    state.velocity += state.acceleration * dt;
    if (!recovery_state_valid(state, limits)) {break;}
    prefix.push_back(state);
    int steps = 0;
    double recovery = 0.;
    if (std::abs(state.acceleration) > 1e-12) {
      const double room = state.acceleration > 0. ?
        limits.velocity_max - state.velocity : state.velocity - limits.velocity_min;
      const double bound = state.acceleration > 0. ?
        -limits.native_input_min : limits.native_input_max;
      if (room < -1e-9 || bound <= 0.) {continue;}
      const int maximum = static_cast<int>(std::floor(std::min(
        static_cast<double>(horizon - p),
        1. + 2. * std::max(0., room) / (std::abs(state.acceleration) * dt)) + 1e-10));
      const int minimum = static_cast<int>(std::ceil(
        std::abs(state.acceleration) / (bound * dt) - 1e-10));
      if (maximum < 1 || maximum < minimum) {continue;}
      steps = maximum;
      recovery = -state.acceleration / (steps * dt);
    }
    selected = p;
    result.recovery_steps = steps;
    result.recovery_input = recovery;
  }
  if (selected == 0) {return result;}
  result.active_input_steps = selected;
  result.states.assign(prefix.begin(), prefix.begin() + selected);
  state = result.states.back();
  for (int k = selected; k < horizon; ++k) {
    state.acceleration += (k - selected < result.recovery_steps ? result.recovery_input : 0.) * dt;
    state.velocity += state.acceleration * dt;
    if (!recovery_state_valid(state, limits)) {return RecoveryRollout{};}
    result.states.push_back(state);
  }
  result.valid = std::abs(state.acceleration) <= 1e-9;
  return result;
}

struct ZeroFirResponse
{
  std::vector<double> step;
  std::vector<double> integrated;
  std::vector<AxisState> free_states;
  std::vector<FeasibleInterval> duration_intervals;
  std::vector<FeasibleInterval> input_intervals;
  int horizon{0};
  double dt{0.};
  bool monotone_integral{false};
  bool valid{false};
};

inline double fir_integral_at(const ZeroFirResponse & response, int k)
{
  return k <= 0 ? 0. : response.integrated[k];
}

inline double fir_step_at(const ZeroFirResponse & response, int k)
{
  return k < 0 ? 0. : response.step[k];
}

// Eliminate q from the full-tail velocity/acceleration inequalities for each N.
// These scalar intervals supply the input domain and acceleration checks; no
// trajectory is generated for each combination of duration and sampled input.
inline ZeroFirResponse prepare_zero_fir_response(
  const AxisState & initial, const AxisLimits & limits,
  const std::vector<double> & coefficients, const std::vector<double> & initial_history,
  double dt, int horizon)
{
  ZeroFirResponse result;
  if (coefficients.empty() || initial_history.size() + 1 != coefficients.size() ||
    !std::isfinite(initial.velocity) || !std::isfinite(dt) || dt <= 0. || horizon < 1) {
    return result;
  }
  result.horizon = horizon;
  result.dt = dt;
  const int count = horizon + static_cast<int>(coefficients.size()) - 1;
  result.integrated.push_back(0.);
  auto history = initial_history;
  double sum = 0., v = initial.velocity;
  result.monotone_integral = true;
  for (int k = 0; k < count; ++k) {
    if (k < static_cast<int>(coefficients.size())) {sum += coefficients[k];}
    if (!std::isfinite(sum)) {return ZeroFirResponse{};}
    result.step.push_back(sum);
    result.integrated.push_back(result.integrated.back() + sum);
    result.monotone_integral &= sum >= 0.;
    const double acceleration = fir_acceleration(coefficients, history, 0.);
    if (!std::isfinite(acceleration)) {return ZeroFirResponse{};}
    v += dt * acceleration;
    result.free_states.push_back({v, acceleration});
    push_fir_input(history, 0.);
  }
  result.duration_intervals.resize(horizon + 1);
  for (int n = 1; n <= horizon; ++n) {
    FeasibleInterval interval{limits.native_input_min, limits.native_input_max, true};
    for (int k = 0; k < count && interval.feasible; ++k) {
      const double ag = result.step[k] - fir_step_at(result, k - n);
      const double vg = dt * (result.integrated[k + 1] - fir_integral_at(result, k + 1 - n));
      constrain_input(interval, result.free_states[k].velocity, vg,
        limits.velocity_min, limits.velocity_max);
      constrain_input(interval, result.free_states[k].acceleration, ag,
        limits.acceleration_min, limits.acceleration_max);
    }
    result.duration_intervals[n] = interval;
  }
  result.input_intervals = merge_input_intervals(result.duration_intervals);
  result.valid = true;
  return result;
}

struct InputStepInterval
{
  int lower{1};
  int upper{0};
};

// v[k,N] = b[k] + q*dt*(R[k]-R[max(k-N,0)]). Invert the monotone R table.
inline InputStepInterval zero_fir_velocity_steps(
  const ZeroFirResponse & response, const AxisLimits & limits, double input)
{
  if (!response.valid || !std::isfinite(input)) {return {};}
  InputStepInterval interval{1, response.horizon};
  // Nonmonotone step-integrals can have disconnected duration sets. In that
  // case retain the full domain here and use the exact scalar intervals below.
  if (!response.monotone_integral) {return interval;}
  const double low = limits.velocity_min - 1e-10, high = limits.velocity_max + 1e-10;
  for (std::size_t index = 0; index < response.free_states.size(); ++index) {
    const double free = response.free_states[index].velocity;
    if (input == 0.) {
      if (free < low || free > high) {return {};}
      continue;
    }
    const int k = static_cast<int>(index) + 1;
    const double r = response.integrated[k];
    const double first = r + (free - low) / (input * response.dt);
    const double second = r + (free - high) / (input * response.dt);
    const double lower = std::min(first, second), upper = std::max(first, second);
    if (lower > r || upper < 0.) {return {};}
    if (lower > 0.) {
      const auto at = std::lower_bound(response.integrated.begin(), response.integrated.end(), lower);
      interval.upper = std::min(interval.upper, k - static_cast<int>(at - response.integrated.begin()));
    }
    if (upper < r) {
      const auto at = std::upper_bound(response.integrated.begin(), response.integrated.end(), upper);
      interval.lower = std::max(interval.lower, k - static_cast<int>(at - response.integrated.begin()) + 1);
    }
    if (interval.lower > interval.upper) {return {};}
  }
  return interval;
}

inline RecoveryRollout longest_zero_fir_input(
  const ZeroFirResponse & response, const AxisLimits & limits, double input)
{
  RecoveryRollout result;
  if (!response.valid || !std::isfinite(input) || input < limits.native_input_min - 1e-12 ||
    input > limits.native_input_max + 1e-12) {return result;}
  const auto steps = zero_fir_velocity_steps(response, limits, input);
  int selected = 0;
  for (int n = steps.upper; n >= steps.lower; --n) {
    const auto & interval = response.duration_intervals[n];
    if (interval.feasible && input >= interval.lower - 1e-12 && input <= interval.upper + 1e-12) {
      selected = n;
      break;
    }
  }
  if (selected == 0) {return result;}
  result.active_input_steps = selected;
  for (int k = 0; k < static_cast<int>(response.free_states.size()); ++k) {
    const AxisState state{
      response.free_states[k].velocity + input * response.dt *
      (response.integrated[k + 1] - fir_integral_at(response, k + 1 - selected)),
      response.free_states[k].acceleration + input *
      (response.step[k] - fir_step_at(response, k - selected))};
    if (!recovery_state_valid(state, limits)) {return RecoveryRollout{};}
    if (k < response.horizon) {result.states.push_back(state);}
  }
  result.valid = true;
  return result;
}

}  // namespace f_dwa_controller
#endif  // F_DWA_CONTROLLER__SATURATION_INPUT_DYNAMICS_HPP_
