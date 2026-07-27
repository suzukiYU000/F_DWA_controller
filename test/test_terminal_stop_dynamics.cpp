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

#include <cmath>

#include "f_dwa_controller/terminal_stop_dynamics.hpp"

namespace f_dwa_controller
{

namespace
{

void expect_directional_bounds(
  const StopSequence & sequence,
  const bool positive_direction,
  const AxisLimits & limits)
{
  ASSERT_TRUE(sequence.feasible);
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

TEST(TerminalStopDynamics, JerkClearsStateInsideVelocityCaptureTube)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};

  const StopSequence sequence =
    generate_jerk_stop_sequence(
    AxisState{0.005, 0.3}, limits, 0.03, 200, 0.01);

  EXPECT_TRUE(sequence.feasible);
  EXPECT_TRUE(sequence.terminal_state_cleared);
  EXPECT_TRUE(sequence.states.empty());
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
