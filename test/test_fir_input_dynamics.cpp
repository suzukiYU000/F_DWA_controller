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

#include <vector>

#include "f_dwa_controller/fir_input_dynamics.hpp"

namespace f_dwa_controller
{

TEST(FirInputDynamics, ComputesConvolutionAndPushesHistory)
{
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  std::vector<double> history{0.4, -0.2};

  EXPECT_NEAR(
    fir_acceleration(coefficients, history, 1.0), 0.58, 1.0e-12);
  push_fir_input(history, 1.0);
  EXPECT_DOUBLE_EQ(history[0], 1.0);
  EXPECT_DOUBLE_EQ(history[1], 0.4);
}

TEST(FirInputDynamics, HeldProjectionRemainsRecursivelyFeasible)
{
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  std::vector<double> history(coefficients.size() - 1u, 0.0);
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  AxisState state;
  constexpr int kStepCount = 80;

  for (int step_index = 0; step_index < kStepCount; ++step_index) {
    const ProjectedFirStep step =
      project_held_fir_step(
      state, limits, coefficients, history, 1.2, 0.03,
      kStepCount - step_index);
    ASSERT_TRUE(step.feasible);
    state = step.state;
    history = step.history;
    EXPECT_GE(state.velocity, limits.velocity_min - 1.0e-9);
    EXPECT_LE(state.velocity, limits.velocity_max + 1.0e-9);
    EXPECT_GE(state.acceleration, limits.acceleration_min - 1.0e-9);
    EXPECT_LE(state.acceleration, limits.acceleration_max + 1.0e-9);
  }
}

TEST(FirInputDynamics, RejectsUnrecoverableHistoryResponse)
{
  const std::vector<double> coefficients{0.1, 0.9};
  const std::vector<double> history{1.2};
  const AxisLimits limits{-1.2, 1.2, -0.5, 0.5, -1.2, 1.2};

  const FeasibleInterval interval =
    held_fir_input_interval(
    AxisState{}, limits, coefficients, history, 0.03, 1);

  EXPECT_FALSE(interval.feasible);
}

TEST(FirInputDynamics, ProjectedStepRejectsConstraintViolationWithoutClamping)
{
  const AxisState state{0.99, 0.0};
  const AxisLimits limits{0.0, 1.0, -1.0, 1.0, -2.0, 2.0};
  const std::vector<double> coefficients{1.0};
  const std::vector<double> history;

  const ProjectedFirStep step =
    apply_projected_fir_step(
    state, limits, coefficients, history, 2.0, 0.03);

  EXPECT_FALSE(step.feasible);
  EXPECT_DOUBLE_EQ(step.applied_native_input, 2.0);
  EXPECT_GT(step.state.velocity, limits.velocity_max);
}

}  // namespace f_dwa_controller
