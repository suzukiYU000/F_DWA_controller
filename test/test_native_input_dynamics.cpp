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

#include "f_dwa_controller/native_input_dynamics.hpp"

namespace f_dwa_controller
{

TEST(NativeInputDynamics, AccelerationProjectionPreservesVelocityLimit)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  AxisState state{1.19, 0.0};

  const ProjectedAxisStep step =
    project_acceleration_step(state, limits, 1.2, 0.03);

  ASSERT_TRUE(step.feasible);
  EXPECT_NEAR(step.state.velocity, 1.2, 1.0e-12);
  EXPECT_NEAR(step.state.acceleration, 1.0 / 3.0, 1.0e-12);
}

TEST(NativeInputDynamics, JerkProjectionPreservesAccelerationAndVelocityLimits)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};
  const AxisState state{1.19, 0.32};

  const ProjectedAxisStep step =
    project_jerk_step(state, limits, 1.57, 0.03);

  ASSERT_TRUE(step.feasible);
  EXPECT_LE(step.state.velocity, 1.2 + 1.0e-12);
  EXPECT_LE(step.state.acceleration, 1.2 + 1.0e-12);
  EXPECT_GE(step.applied_native_input, -1.57 - 1.0e-12);
  EXPECT_LE(step.applied_native_input, 1.57 + 1.0e-12);
}

TEST(NativeInputDynamics, JerkRejectsUnrecoverableOneStepState)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};
  const AxisState state{1.19, 1.19};

  const ProjectedAxisStep step =
    project_jerk_step(state, limits, -1.57, 0.03);

  EXPECT_FALSE(step.feasible);
}

TEST(NativeInputDynamics, JerkUsesAccelerationState)
{
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.57, 1.57};
  const AxisState state{0.4, 0.3};

  const ProjectedAxisStep step =
    project_jerk_step(state, limits, -1.0, 0.03);

  ASSERT_TRUE(step.feasible);
  EXPECT_NEAR(step.state.acceleration, 0.27, 1.0e-12);
  EXPECT_NEAR(step.state.velocity, 0.4081, 1.0e-12);
}

TEST(NativeInputDynamics, HeldJerkProjectionRemainsRecursivelyFeasible)
{
  const AxisLimits limits{-1.57, 1.57, -1.57, 1.57, -1.57, 1.57};
  AxisState state;

  for (int remaining_steps = 80; remaining_steps > 0; --remaining_steps) {
    const ProjectedAxisStep step =
      project_held_jerk_step(
      state, limits, -1.57, 0.03, remaining_steps);
    ASSERT_TRUE(step.feasible);
    state = step.state;
    EXPECT_GE(state.velocity, limits.velocity_min - 1.0e-10);
    EXPECT_LE(state.velocity, limits.velocity_max + 1.0e-10);
    EXPECT_GE(state.acceleration, limits.acceleration_min - 1.0e-10);
    EXPECT_LE(state.acceleration, limits.acceleration_max + 1.0e-10);
  }
}

TEST(NativeInputDynamics, UniformSamplesPreserveConfiguredBudget)
{
  const FeasibleInterval interval{-0.5, 1.0, true};

  const std::vector<double> samples = uniform_samples(interval, 11);

  ASSERT_EQ(samples.size(), 11u);
  EXPECT_DOUBLE_EQ(samples.front(), -0.5);
  EXPECT_DOUBLE_EQ(samples.back(), 1.0);
}

}  // namespace f_dwa_controller
