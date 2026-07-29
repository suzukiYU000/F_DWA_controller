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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "f_dwa_controller/fir_input_dynamics.hpp"
#include "f_dwa_controller/terminal_stop_dynamics.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr double kReferenceBoundTightening = 2.0e-7;

AxisLimits reference_directional_limits(
  const AxisLimits & limits,
  const bool positive_direction)
{
  AxisLimits directional = limits;
  if (positive_direction) {
    directional.velocity_min = 0.0;
  } else {
    directional.velocity_max = 0.0;
  }
  directional.velocity_min += kReferenceBoundTightening;
  directional.velocity_max -= kReferenceBoundTightening;
  directional.acceleration_min += kReferenceBoundTightening;
  directional.acceleration_max -= kReferenceBoundTightening;
  directional.native_input_min += kReferenceBoundTightening;
  directional.native_input_max -= kReferenceBoundTightening;
  return directional;
}

bool reference_state_is_terminal(
  const AxisState & state,
  const double terminal_threshold)
{
  return std::abs(state.velocity) <= terminal_threshold &&
         std::abs(state.acceleration) <= terminal_threshold;
}

bool reference_history_is_terminal(
  const std::vector<double> & history,
  const double terminal_threshold)
{
  return std::all_of(
    history.begin(), history.end(),
    [terminal_threshold](const double value) {
      return std::abs(value) <= terminal_threshold;
    });
}

bool reference_zero_input_reaches_terminal(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & initial_history,
  const double time_step,
  const int maximum_steps,
  const double terminal_threshold)
{
  AxisState state = initial_state;
  std::vector<double> history = initial_history;
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    if (!apply_projected_fir_step_in_place(
        state, limits, coefficients, history, 0.0, time_step))
    {
      return false;
    }
    if (reference_state_is_terminal(state, terminal_threshold) &&
      reference_history_is_terminal(history, terminal_threshold))
    {
      return true;
    }
  }
  return false;
}

StopSequence generate_reference_fir_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  const double time_step,
  const int maximum_steps,
  const double stop_velocity_threshold,
  const bool record_fir_histories)
{
  StopSequence sequence;
  if (!std::isfinite(initial_state.velocity) ||
    !std::isfinite(initial_state.acceleration) ||
    !std::isfinite(time_step) || time_step <= 0.0 ||
    maximum_steps <= 0 ||
    !std::isfinite(stop_velocity_threshold) ||
    stop_velocity_threshold <= 0.0 ||
    limits.velocity_min > 0.0 ||
    limits.velocity_max < 0.0 ||
    coefficients.empty() ||
    history.size() + 1u != coefficients.size() ||
    std::abs(coefficients.front()) <= 1.0e-12)
  {
    return sequence;
  }
  if (reference_state_is_terminal(
      initial_state, stop_velocity_threshold) &&
    reference_history_is_terminal(history, stop_velocity_threshold))
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
      fir_acceleration(coefficients, history, 0.0);
  }
  const bool positive_direction =
    direction_state.velocity > 0.0 ||
    (direction_state.velocity == 0.0 &&
    direction_state.acceleration >= 0.0);
  const AxisLimits stop_limits =
    reference_directional_limits(limits, positive_direction);
  AxisState state = initial_state;
  std::vector<double> current_history = history;
  sequence.native_inputs.reserve(static_cast<std::size_t>(maximum_steps));
  sequence.states.reserve(static_cast<std::size_t>(maximum_steps));
  bool zero_input_tail_active = false;
  for (int step_index = 0; step_index < maximum_steps; ++step_index) {
    const int lookahead_steps =
      std::min(
      maximum_steps - step_index,
      static_cast<int>(coefficients.size()));
    FeasibleInterval feasible_input;
    if (zero_input_tail_active ||
      reference_zero_input_reaches_terminal(
        state, stop_limits, coefficients, current_history,
        time_step, lookahead_steps, stop_velocity_threshold))
    {
      zero_input_tail_active = true;
      feasible_input.lower = 0.0;
      feasible_input.upper = 0.0;
      feasible_input.feasible = true;
    } else {
      feasible_input =
        held_fir_input_interval(
        state, stop_limits, coefficients, current_history,
        time_step, lookahead_steps);
    }
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
    double requested_input = 0.0;
    if (!zero_input_tail_active) {
      const double requested_acceleration =
        positive_direction ? acceleration_lower : acceleration_upper;
      const double free_acceleration =
        fir_acceleration(coefficients, current_history, 0.0);
      requested_input =
        (requested_acceleration - free_acceleration) /
        coefficients.front();
    }
    const double applied_native_input =
      std::clamp(
      requested_input, feasible_input.lower, feasible_input.upper);
    if (!apply_projected_fir_step_in_place(
        state, stop_limits, coefficients, current_history,
        applied_native_input, time_step))
    {
      return sequence;
    }
    sequence.native_inputs.push_back(applied_native_input);
    sequence.states.push_back(state);
    if (record_fir_histories) {
      sequence.fir_histories.push_back(current_history);
    }
    if (reference_state_is_terminal(state, stop_velocity_threshold) &&
      reference_history_is_terminal(
        current_history, stop_velocity_threshold))
    {
      sequence.feasible = true;
      sequence.terminal_state_cleared = true;
      return sequence;
    }
  }
  return sequence;
}

std::vector<double> f8_coefficients()
{
  return {
    0.035777752054243751, 0.045878186521488042,
    0.062570166323592399, 0.08041805329356351,
    0.097354323429160852, 0.11144922512678306,
    0.12092495177348041, 0.12436852339294127,
    0.12095127393441317, 0.11058504185223869,
    0.093965326304448094, 0.07251722845311262,
    0.048228323572666855, 0.023398363395377646,
    0.00034114993982012419, -0.018872755721100667,
    -0.032757703647982202, -0.040433615647479569,
    -0.041877094771567765, -0.037755176814922603,
    -0.029318076048936739, -0.018245918418712214,
    -0.0063632259584256905, 0.0046272774558559571,
    0.013345914377498576, 0.018885917438179037,
    0.020891029358078735, 0.019566571466093498,
    0.015582497582436234, 0.0099104480557657502,
    0.0036435108533972907, -0.0021643057581709852,
    -0.006691328081410014, -0.0094021045255586987,
    -0.010148404350806287, -0.009122316629276055,
    -0.0067781407345995472, -0.0037410844795386844,
    -0.00065405392845252746, 0.0019347283013268341,
    0.0036468558098519698, 0.0043335642412933332,
    0.0040642521005098062, 0.0030832199513739921,
    0.0017309988770190827, 0.00035063028092987083};
}

void expect_stop_sequences_near(
  const StopSequence & reference,
  const StopSequence & optimized,
  const double tolerance)
{
  ASSERT_EQ(reference.feasible, optimized.feasible);
  ASSERT_EQ(
    reference.terminal_state_cleared,
    optimized.terminal_state_cleared);
  ASSERT_EQ(reference.native_inputs.size(), optimized.native_inputs.size());
  ASSERT_EQ(reference.states.size(), optimized.states.size());
  ASSERT_EQ(reference.fir_histories.size(), optimized.fir_histories.size());
  for (std::size_t index = 0; index < reference.states.size(); ++index) {
    EXPECT_NEAR(
      reference.native_inputs[index],
      optimized.native_inputs[index], tolerance);
    EXPECT_NEAR(
      reference.states[index].velocity,
      optimized.states[index].velocity, tolerance);
    EXPECT_NEAR(
      reference.states[index].acceleration,
      optimized.states[index].acceleration, tolerance);
  }
  for (std::size_t step_index = 0;
    step_index < reference.fir_histories.size(); ++step_index)
  {
    ASSERT_EQ(
      reference.fir_histories[step_index].size(),
      optimized.fir_histories[step_index].size());
    for (std::size_t tap_index = 0;
      tap_index < reference.fir_histories[step_index].size(); ++tap_index)
    {
      EXPECT_NEAR(
        reference.fir_histories[step_index][tap_index],
        optimized.fir_histories[step_index][tap_index], tolerance);
    }
  }
}

void expect_directional_bounds(
  const StopSequence & sequence,
  const bool positive_direction,
  const AxisLimits & limits)
{
  const double last_velocity =
    sequence.states.empty() ? 0.0 : sequence.states.back().velocity;
  const double last_acceleration =
    sequence.states.empty() ? 0.0 : sequence.states.back().acceleration;
  ASSERT_TRUE(sequence.feasible)
    << "steps=" << sequence.states.size()
    << " last_velocity=" << last_velocity
    << " last_acceleration=" << last_acceleration;
  ASSERT_TRUE(sequence.terminal_state_cleared);
  for (const AxisState & state : sequence.states) {
    EXPECT_GE(state.velocity, limits.velocity_min - 1.0e-7);
    EXPECT_LE(state.velocity, limits.velocity_max + 1.0e-7);
    EXPECT_GE(state.acceleration, limits.acceleration_min - 1.0e-7);
    EXPECT_LE(state.acceleration, limits.acceleration_max + 1.0e-7);
    if (positive_direction) {
      EXPECT_GE(state.velocity, -1.0e-7);
    } else {
      EXPECT_LE(state.velocity, 1.0e-7);
    }
  }
  if (!sequence.states.empty()) {
    EXPECT_LE(std::abs(sequence.states.back().velocity), 0.01);
    EXPECT_LE(std::abs(sequence.states.back().acceleration), 0.01);
  }
  if (!sequence.fir_histories.empty()) {
    for (const double value : sequence.fir_histories.back()) {
      EXPECT_LE(std::abs(value), 0.01);
    }
  }
}

}  // namespace

TEST(TerminalStopDynamics, AccelerationStopsWithoutCrossingZero)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};

  const StopSequence sequence =
    generate_acceleration_stop_sequence(
    AxisState{1.2, 0.0}, limits, 0.03, 200, 0.01);

  expect_directional_bounds(sequence, true, limits);
}

TEST(TerminalStopDynamics, AccelerationClearsInputInsideVelocityCaptureTube)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};

  const StopSequence sequence =
    generate_acceleration_stop_sequence(
    AxisState{0.005, 0.3}, limits, 0.03, 200, 0.01);

  ASSERT_TRUE(sequence.feasible);
  ASSERT_TRUE(sequence.terminal_state_cleared);
  ASSERT_EQ(sequence.states.size(), 1u);
  EXPECT_DOUBLE_EQ(sequence.native_inputs.back(), 0.0);
  EXPECT_DOUBLE_EQ(sequence.states.back().velocity, 0.005);
  EXPECT_DOUBLE_EQ(sequence.states.back().acceleration, 0.0);
}

TEST(TerminalStopDynamics, JerkStopsPositiveMotionAndClearsState)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};

  const StopSequence sequence =
    generate_jerk_stop_sequence(
    AxisState{1.0, 0.3}, limits, 0.03, 200, 0.01);

  expect_directional_bounds(sequence, true, limits);
}

TEST(TerminalStopDynamics, JerkStopsNegativeRotationAndClearsState)
{
  const AxisLimits limits{-1.57, 1.57, -1.57, 1.57, -1.57, 1.57};

  const StopSequence sequence =
    generate_jerk_stop_sequence(
    AxisState{-1.0, -0.2}, limits, 0.03, 200, 0.01);

  expect_directional_bounds(sequence, false, limits);
}

TEST(TerminalStopDynamics, JerkReleasesStateInsideVelocityCaptureTube)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};

  const StopSequence sequence =
    generate_jerk_stop_sequence(
    AxisState{0.005, 0.3}, limits, 0.03, 200, 0.01);

  EXPECT_TRUE(sequence.feasible);
  EXPECT_TRUE(sequence.terminal_state_cleared);
  ASSERT_FALSE(sequence.states.empty());
  EXPECT_LE(std::abs(sequence.states.back().velocity), 0.01);
  EXPECT_LE(std::abs(sequence.states.back().acceleration), 0.01);
}

TEST(TerminalStopDynamics, JerkRandomizedStopsRetainTerminalState)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};
  std::mt19937 generator(20260728u);
  std::uniform_real_distribution<double> speed_distribution(0.3, 1.1);
  std::uniform_real_distribution<double> acceleration_distribution(-0.5, 0.5);

  for (int trial_index = 0; trial_index < 500; ++trial_index) {
    const bool positive_direction = trial_index % 2 == 0;
    const double direction = positive_direction ? 1.0 : -1.0;
    const StopSequence sequence =
      generate_jerk_stop_sequence(
      AxisState{
        direction * speed_distribution(generator),
        acceleration_distribution(generator)},
      limits, 0.03, 267, 0.01);

    SCOPED_TRACE(trial_index);
    expect_directional_bounds(sequence, positive_direction, limits);
  }
}

TEST(TerminalStopDynamics, FirStopsWithFilteredAcceleration)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> history(coefficients.size() - 1u, 0.0);

  const StopSequence sequence =
    generate_fir_stop_sequence(
    AxisState{1.0, 0.0}, limits, coefficients, history,
    0.03, 267, 0.01);

  expect_directional_bounds(sequence, true, limits);
  EXPECT_EQ(sequence.native_inputs.size(), sequence.states.size());
}

TEST(TerminalStopDynamics, FirDrainsHistoryInsideVelocityCaptureTube)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> history{-0.1, 0.0};

  const StopSequence sequence =
    generate_fir_stop_sequence(
    AxisState{0.005, -0.03}, limits, coefficients, history,
    0.03, 267, 0.01, true);

  ASSERT_TRUE(sequence.feasible);
  ASSERT_TRUE(sequence.terminal_state_cleared);
  ASSERT_FALSE(sequence.states.empty());
  ASSERT_EQ(sequence.fir_histories.size(), sequence.states.size());
  EXPECT_LE(std::abs(sequence.states.back().velocity), 0.01);
  EXPECT_LE(std::abs(sequence.states.back().acceleration), 0.01);
  for (const double value : sequence.fir_histories.back()) {
    EXPECT_LE(std::abs(value), 0.01);
  }
}

TEST(TerminalStopDynamics, FirHistoryRecordingDoesNotChangeDynamics)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> history{0.15, -0.05};

  const StopSequence recorded =
    generate_fir_stop_sequence(
    AxisState{0.8, 0.04}, limits, coefficients, history,
    0.03, 267, 0.01, true);
  const StopSequence lightweight =
    generate_fir_stop_sequence(
    AxisState{0.8, 0.04}, limits, coefficients, history,
    0.03, 267, 0.01, false);

  ASSERT_TRUE(recorded.feasible);
  ASSERT_TRUE(lightweight.feasible);
  ASSERT_EQ(recorded.native_inputs.size(), lightweight.native_inputs.size());
  ASSERT_EQ(recorded.states.size(), lightweight.states.size());
  EXPECT_EQ(recorded.fir_histories.size(), recorded.states.size());
  EXPECT_TRUE(lightweight.fir_histories.empty());
  for (std::size_t index = 0; index < recorded.states.size(); ++index) {
    EXPECT_DOUBLE_EQ(
      recorded.native_inputs[index], lightweight.native_inputs[index]);
    EXPECT_DOUBLE_EQ(
      recorded.states[index].velocity,
      lightweight.states[index].velocity);
    EXPECT_DOUBLE_EQ(
      recorded.states[index].acceleration,
      lightweight.states[index].acceleration);
  }
}

TEST(TerminalStopDynamics, PreparedFirPredictionMatchesF8Reference)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients = f8_coefficients();
  std::vector<double> history(coefficients.size() - 1u, 0.0);
  for (std::size_t index = 0; index < history.size(); ++index) {
    history[index] =
      0.15 * std::sin(0.37 * static_cast<double>(index));
  }
  const AxisState initial_state{
    0.8, fir_acceleration(coefficients, history, 0.2)};

  const StopSequence reference =
    generate_reference_fir_stop_sequence(
    initial_state, limits, coefficients, history,
    0.03, 267, 0.01, true);
  const StopSequence optimized =
    generate_fir_stop_sequence(
    initial_state, limits, coefficients, history,
    0.03, 267, 0.01, true);

  expect_stop_sequences_near(reference, optimized, 1.0e-12);
}

TEST(TerminalStopDynamics, PreparedFirPredictionMatchesRandomReferences)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients = f8_coefficients();
  const FirStopCoefficientResponse coefficient_response =
    prepare_fir_stop_coefficient_response(coefficients, 0.03);
  ASSERT_TRUE(coefficient_response.valid);
  std::mt19937 generator(20260728u);
  std::uniform_real_distribution<double> history_distribution(-1.2, 1.2);
  std::uniform_real_distribution<double> velocity_distribution(-1.1, 1.1);
  std::uniform_real_distribution<double> input_distribution(-1.2, 1.2);

  for (int trial_index = 0; trial_index < 1000; ++trial_index) {
    std::vector<double> history(coefficients.size() - 1u);
    for (double & value : history) {
      value = history_distribution(generator);
    }
    const AxisState initial_state{
      velocity_distribution(generator),
      fir_acceleration(
        coefficients, history, input_distribution(generator))};
    const StopSequence reference =
      generate_reference_fir_stop_sequence(
      initial_state, limits, coefficients, history,
      0.03, 267, 0.01, false);
    const StopSequence optimized =
      generate_fir_stop_sequence(
      initial_state, limits, coefficients, history,
      0.03, 267, 0.01, false);
    const StopSequence cached =
      generate_fir_stop_sequence(
      initial_state, limits, coefficients, history,
      0.03, 267, 0.01, false, &coefficient_response);

    SCOPED_TRACE(trial_index);
    expect_stop_sequences_near(reference, optimized, 1.0e-12);
    ASSERT_EQ(cached.feasible, optimized.feasible);
    ASSERT_EQ(
      cached.terminal_state_cleared,
      optimized.terminal_state_cleared);
    ASSERT_EQ(cached.native_inputs.size(), optimized.native_inputs.size());
    ASSERT_EQ(cached.states.size(), optimized.states.size());
    for (std::size_t step_index = 0u;
      step_index < optimized.states.size(); ++step_index)
    {
      EXPECT_DOUBLE_EQ(
        cached.native_inputs[step_index],
        optimized.native_inputs[step_index]);
      EXPECT_DOUBLE_EQ(
        cached.states[step_index].velocity,
        optimized.states[step_index].velocity);
      EXPECT_DOUBLE_EQ(
        cached.states[step_index].acceleration,
        optimized.states[step_index].acceleration);
    }
  }
}

TEST(TerminalStopDynamics, PreparedFirPredictionPreservesBoundaryDecisions)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients = f8_coefficients();
  const std::vector<double> velocities{
    0.01 - 1.0e-13, 0.01 + 1.0e-13,
    -0.01 + 1.0e-13, -0.01 - 1.0e-13,
    1.2 - 3.0e-7, -1.2 + 3.0e-7};

  for (std::size_t velocity_index = 0;
    velocity_index < velocities.size(); ++velocity_index)
  {
    std::vector<double> history(coefficients.size() - 1u);
    for (std::size_t tap_index = 0;
      tap_index < history.size(); ++tap_index)
    {
      const double direction =
        (tap_index + velocity_index) % 2u == 0u ? 1.0 : -1.0;
      history[tap_index] =
        direction * (1.2 - 1.0e-12 * static_cast<double>(tap_index));
    }
    const AxisState initial_state{
      velocities[velocity_index],
      fir_acceleration(coefficients, history, 0.0)};
    const StopSequence reference =
      generate_reference_fir_stop_sequence(
      initial_state, limits, coefficients, history,
      0.03, 267, 0.01, false);
    const StopSequence optimized =
      generate_fir_stop_sequence(
      initial_state, limits, coefficients, history,
      0.03, 267, 0.01, false);

    SCOPED_TRACE(velocity_index);
    expect_stop_sequences_near(reference, optimized, 1.0e-12);
  }
}

TEST(TerminalStopDynamics, RejectsStateThatCannotRespectDirectionalBounds)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};

  const StopSequence sequence =
    generate_jerk_stop_sequence(
    AxisState{0.001, -1.2}, limits, 0.03, 200, 0.0001);

  EXPECT_FALSE(sequence.feasible);
}

}  // namespace f_dwa_controller
