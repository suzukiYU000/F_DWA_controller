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

FeasibleInterval reference_one_tick_fir_interval(
  const AxisState & state, const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history, const double dt, const int steps)
{
  // Brute-force two convolutions: q=0 throughout, and q=1 for one tick.
  // Their difference is the affine response to the single committed input.
  auto zero_history = history;
  auto one_history = history;
  double zero_velocity = state.velocity;
  double one_velocity = state.velocity;
  FeasibleInterval result;
  result.lower = limits.native_input_min;
  result.upper = limits.native_input_max;
  auto constrain = [&result](double base, double slope, double low, double high) {
      if (std::abs(slope) <= 1.0e-12) {
        return base >= low - 1.0e-9 && base <= high + 1.0e-9;
      }
      double a = (low - base) / slope;
      double b = (high - base) / slope;
      if (a > b) {std::swap(a, b);}
      result.lower = std::max(result.lower, a);
      result.upper = std::min(result.upper, b);
      return result.lower <= result.upper;
    };
  for (int index = 0; index < steps; ++index) {
    const double one_input = index == 0 ? 1.0 : 0.0;
    const double zero_acceleration = fir_acceleration(coefficients, zero_history, 0.0);
    const double one_acceleration = fir_acceleration(coefficients, one_history, one_input);
    zero_velocity += dt * zero_acceleration;
    one_velocity += dt * one_acceleration;
    if (!constrain(zero_acceleration, one_acceleration - zero_acceleration,
        limits.acceleration_min, limits.acceleration_max) ||
      !constrain(zero_velocity, one_velocity - zero_velocity,
        limits.velocity_min, limits.velocity_max))
    {
      return result;
    }
    push_fir_input(zero_history, 0.0);
    push_fir_input(one_history, one_input);
  }
  result.feasible = true;
  return result;
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
        reference_one_tick_fir_interval(
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

TEST(TerminalStopDynamics, JerkTerminalStateKeepsExactZeroAndHoldWithinLimit)
{
  const AxisLimits limits{-0.8, 0.8, -1.6, 1.6, -3.2, 3.2};
  constexpr double time_step = 0.05;
  constexpr double initial_speed = 0.008060001544707356;

  for (const double direction : {-1.0, 1.0}) {
    const StopSequence sequence =
      generate_jerk_stop_sequence(
      AxisState{direction * initial_speed, 0.0}, limits,
      time_step, 200, 0.01);

    SCOPED_TRACE(direction);
    ASSERT_TRUE(sequence.feasible);
    ASSERT_TRUE(sequence.terminal_state_cleared);
    ASSERT_FALSE(sequence.states.empty());
    const AxisState & terminal = sequence.states.back();
    const double zero_acceleration = -terminal.velocity / time_step;
    const double zero_transition_jerk =
      (zero_acceleration - terminal.acceleration) / time_step;
    const double zero_hold_jerk = -zero_acceleration / time_step;
    EXPECT_LT(std::abs(zero_transition_jerk), 3.2);
    EXPECT_LT(std::abs(zero_hold_jerk), 3.2);
  }
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

TEST(TerminalStopDynamics, FirNinetyTwoTapsStopAfterOneCommittedInput)
{
  // Minimum-phase Hamming FIR: 20 Hz sampling, 3 Hz cutoff, design length 183.
  // The old held-input certificate stalled at 0.2129688 m/s in this condition.
  const std::vector<double> coefficients{
    0.05692685418449837, 0.13430856494579593, 0.21965595248916325,
    0.27334457901068065, 0.26194939481034824, 0.17918440614514183,
    0.054721087062184544, -0.05836069067876422, -0.11207061978549453,
    -0.09093800963272183, -0.020569254747264995, 0.04907963079601853,
    0.07516913297991097, 0.04762324500887386, -0.0077580504583894,
    -0.04987883075684019, -0.051488132626854415, -0.016452165627945672,
    0.026070005177403682, 0.0447025263529043, 0.02852563172768648,
    -0.006926994646468927, -0.033490883677996776, -0.032288889570255684,
    -0.006874794883569318, 0.021242793241559865, 0.030575173263903164,
    0.015671407812093214, -0.009902981379193121, -0.02554926444595089,
    -0.020107448138431776, 0.0004255635806368002, 0.018890510195472207,
    0.021098683420987956, 0.006653851786574757, -0.011821815579407466,
    -0.019524229548184967, -0.011257925456603092, 0.005208930280832841,
    0.016252393225560567, 0.013579961066115956, 0.0003803992392388165,
    -0.012078316166259302, -0.013975119788112288, -0.004627739806774918,
    0.007659305771770244, 0.012891405869304682, 0.007444124675457289,
    -0.003522692228127822, -0.010801396321085321, -0.008887702760796604,
    2.8677836656794727e-06, 0.008178800813094704, 0.009138167612370532,
    0.0026985674878754534, -0.0053996465579810886, -0.00849696019315178,
    -0.004493534177667877, 0.002790991417893663, 0.007224679101816603,
    0.005453004715258049, -0.0005775814774452459, -0.005624546472445005,
    -0.005676361293798506, -0.0011206846578592383, 0.003932939614576744,
    0.005334083925058791, 0.0022769321398520416, -0.0023413942920398004,
    -0.004622350476070083, -0.0029291170195433836, 0.0009926358269890163,
    0.0037260147285057125, 0.003139305572020165, 4.881781527724068e-05,
    -0.0027838033523059306, -0.0030289726173166644, -0.0007528125100271116,
    0.0018971569831507966, 0.0027005137590718656, 0.0011723210499503004,
    -0.0011539924179777348, -0.0022503944193074097, -0.0013609034466323428,
    0.0005669471974272015, 0.001792965359275627, 0.0013706843679414436,
    -0.0001515307494578204, -0.0013631129073628189, -0.0012751839533591206,
    -0.00011744661018870412, 0.001000366922240639,
  };
  const AxisLimits limits{0.0, 1.5, -1.5, 1.5, -1.2, 1.2};
  for (const double velocity : {0.0, 0.1, 0.2, 0.2129688127865, 0.3, 0.5, 1.0, 1.4}) {
    for (const double input : {0.0, 0.24, 1.2}) {
      SCOPED_TRACE(velocity);
      SCOPED_TRACE(input);
      std::vector<double> history(coefficients.size() - 1u, 0.0);
      AxisState state{velocity + 0.05 * coefficients.front() * input,
        coefficients.front() * input};
      push_fir_input(history, input);
      const StopSequence sequence = generate_fir_stop_sequence(
        state, limits, coefficients, history, 0.05, 240, 0.01, true);
      ASSERT_TRUE(sequence.feasible);
      ASSERT_TRUE(sequence.terminal_state_cleared);
      ASSERT_LE(sequence.states.size(), 240u);
      for (std::size_t index = 0; index < sequence.states.size(); ++index) {
        const double q = sequence.native_inputs[index];
        EXPECT_LE(std::abs(q), 1.2);
        state.acceleration = fir_acceleration(coefficients, history, q);
        state.velocity += 0.05 * state.acceleration;
        push_fir_input(history, q);
        EXPECT_NEAR(state.acceleration, sequence.states[index].acceleration, 1.0e-12);
        EXPECT_NEAR(state.velocity, sequence.states[index].velocity, 1.0e-12);
        EXPECT_EQ(history, sequence.fir_histories[index]);
        EXPECT_GE(state.velocity, -1.0e-12);
        EXPECT_LE(state.velocity, 1.5 + 1.0e-12);
        EXPECT_LE(std::abs(state.acceleration), 1.5 + 1.0e-12);
      }
      EXPECT_LE(std::abs(state.velocity), 0.01);
      EXPECT_LE(std::abs(state.acceleration), 0.01);
      for (const double value : history) {
        EXPECT_LE(std::abs(value), 0.01);
      }
    }
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

TEST(TerminalStopDynamics, FirAngularStopMayCrossZeroBeforeSettling)
{
  const AxisLimits limits{-0.8, 0.8, -1.6, 1.6, -1.57, 1.57};
  const std::vector<double> coefficients = f8_coefficients();
  std::vector<double> history(coefficients.size() - 1u, -0.4);
  history.front() = -1.57;
  const AxisState state{
    0.005, fir_acceleration(coefficients, history, -1.57)};

  const StopSequence directional = generate_fir_stop_sequence(
    state, limits, coefficients, history, 0.03, 400, 0.01,
    false, nullptr, false);
  const StopSequence angular = generate_fir_stop_sequence(
    state, limits, coefficients, history, 0.03, 400, 0.01,
    false, nullptr, true);

  EXPECT_FALSE(directional.feasible);
  ASSERT_TRUE(angular.feasible);
  ASSERT_TRUE(angular.terminal_state_cleared);
  EXPECT_TRUE(std::any_of(
      angular.states.begin(), angular.states.end(),
      [](const AxisState & value) {return value.velocity < 0.0;}));
  for (const AxisState & value : angular.states) {
    EXPECT_GE(value.velocity, limits.velocity_min);
    EXPECT_LE(value.velocity, limits.velocity_max);
    EXPECT_GE(value.acceleration, limits.acceleration_min);
    EXPECT_LE(value.acceleration, limits.acceleration_max);
  }
  EXPECT_LE(std::abs(angular.states.back().velocity), 0.01);
  EXPECT_LE(std::abs(angular.states.back().acceleration), 0.01);
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
