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
#include <numeric>
#include <random>
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

TEST(FirInputDynamics, InPlaceStepMatchesValueReturningStep)
{
  const AxisState state{0.2, 0.1};
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> history{0.4, -0.2};
  const ProjectedFirStep expected =
    apply_projected_fir_step(
    state, limits, coefficients, history, 0.6, 0.03);
  AxisState in_place_state = state;
  std::vector<double> in_place_history = history;

  const bool feasible = apply_projected_fir_step_in_place(
    in_place_state, limits, coefficients, in_place_history, 0.6, 0.03);

  ASSERT_EQ(feasible, expected.feasible);
  EXPECT_DOUBLE_EQ(in_place_state.velocity, expected.state.velocity);
  EXPECT_DOUBLE_EQ(in_place_state.acceleration, expected.state.acceleration);
  EXPECT_EQ(in_place_history, expected.history);
}

TEST(FirInputDynamics, AffineResponseMatchesBoundaryHeldInputRollouts)
{
  constexpr int kStepCount = 80;
  constexpr double kTimeStep = 0.03;
  const AxisState initial_state{0.17, -0.08};
  const AxisLimits limits{-1.2, 1.2, -1.2, 1.2, -1.2, 1.2};
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> initial_history{0.4, -0.2};
  const HeldFirAffineResponse response =
    prepare_held_fir_affine_response(
    initial_state, limits, coefficients, initial_history,
    kTimeStep, kStepCount);
  ASSERT_TRUE(response.input_interval.feasible);
  const FeasibleInterval delegated_interval =
    held_fir_input_interval(
    initial_state, limits, coefficients, initial_history,
    kTimeStep, kStepCount);
  EXPECT_DOUBLE_EQ(
    delegated_interval.lower, response.input_interval.lower);
  EXPECT_DOUBLE_EQ(
    delegated_interval.upper, response.input_interval.upper);
  EXPECT_EQ(
    delegated_interval.feasible, response.input_interval.feasible);

  const std::vector<double> inputs{
    response.input_interval.lower,
    0.5 * (
      response.input_interval.lower + response.input_interval.upper),
    response.input_interval.upper};
  for (const double input : inputs) {
    std::vector<AxisState> affine_states;
    ASSERT_TRUE(
      sample_held_fir_affine_response(
        response, limits, input, affine_states));
    ASSERT_EQ(affine_states.size(), static_cast<std::size_t>(kStepCount));

    AxisState direct_state = initial_state;
    std::vector<double> direct_history = initial_history;
    for (int step_index = 0;
      step_index < kStepCount; ++step_index)
    {
      ASSERT_TRUE(
        apply_projected_fir_step_in_place(
          direct_state, limits, coefficients, direct_history,
          input, kTimeStep));
      const AxisState & affine_state =
        affine_states[static_cast<std::size_t>(step_index)];
      EXPECT_NEAR(affine_state.velocity, direct_state.velocity, 1.0e-12);
      EXPECT_NEAR(
        affine_state.acceleration, direct_state.acceleration, 1.0e-12);
    }
  }
}

TEST(FirInputDynamics, PulsedAffineResponseTurnsRawInputOffAfterPulse)
{
  const std::vector<double> coefficients{0.5, 0.3, 0.2};
  const std::vector<double> history(coefficients.size() - 1u, 0.0);
  const AxisLimits limits{-10.0, 10.0, -10.0, 10.0, -2.0, 2.0};
  const HeldFirAffineResponse response =
    prepare_pulsed_fir_affine_response(
    AxisState{}, limits, coefficients, history, 1.0, 5, 2);
  ASSERT_TRUE(response.input_interval.feasible);

  std::vector<AxisState> states;
  ASSERT_TRUE(
    sample_held_fir_affine_response(response, limits, 1.0, states));
  ASSERT_EQ(states.size(), 5u);
  const std::vector<double> expected_accelerations{
    0.5, 0.8, 0.5, 0.2, 0.0};
  const std::vector<double> expected_velocities{
    0.5, 1.3, 1.8, 2.0, 2.0};
  for (std::size_t index = 0; index < states.size(); ++index) {
    EXPECT_NEAR(
      states[index].acceleration,
      expected_accelerations[index], 1.0e-12);
    EXPECT_NEAR(
      states[index].velocity, expected_velocities[index], 1.0e-12);
  }
}

TEST(FirInputDynamics, AffineResponseMatchesRandomFortySixTapRollouts)
{
  constexpr int kStepCount = 80;
  constexpr int kTapCount = 46;
  constexpr double kTimeStep = 0.03;
  constexpr int kTrialCount = 100;
  std::mt19937 random_engine(43019u);
  std::uniform_real_distribution<double> coefficient_distribution(
    0.001, 1.0);
  std::uniform_real_distribution<double> state_distribution(-0.4, 0.4);
  const AxisLimits limits{-2.0, 2.0, -1.5, 1.5, -1.2, 1.2};
  int feasible_trial_count = 0;

  for (int trial_index = 0;
    trial_index < kTrialCount; ++trial_index)
  {
    std::vector<double> coefficients(
      static_cast<std::size_t>(kTapCount));
    std::generate(
      coefficients.begin(), coefficients.end(),
      [&]() {return coefficient_distribution(random_engine);});
    const double coefficient_sum =
      std::accumulate(coefficients.begin(), coefficients.end(), 0.0);
    for (double & coefficient : coefficients) {
      coefficient /= coefficient_sum;
    }
    std::vector<double> initial_history(
      static_cast<std::size_t>(kTapCount - 1));
    std::generate(
      initial_history.begin(), initial_history.end(),
      [&]() {return state_distribution(random_engine);});
    const AxisState initial_state{
      state_distribution(random_engine),
      state_distribution(random_engine)};
    const HeldFirAffineResponse response =
      prepare_held_fir_affine_response(
      initial_state, limits, coefficients, initial_history,
      kTimeStep, kStepCount);
    if (!response.input_interval.feasible) {
      continue;
    }
    ++feasible_trial_count;

    std::vector<double> inputs;
    inputs.reserve(11u);
    for (int input_index = 0; input_index < 11; ++input_index) {
      const double ratio =
        static_cast<double>(input_index) / 10.0;
      inputs.push_back(
        response.input_interval.lower +
        (response.input_interval.upper - response.input_interval.lower) *
        ratio);
    }
    ASSERT_EQ(inputs.size(), 11u);
    for (const double input : inputs) {
      std::vector<AxisState> affine_states;
      ASSERT_TRUE(
        sample_held_fir_affine_response(
          response, limits, input, affine_states));
      ASSERT_EQ(
        affine_states.size(), static_cast<std::size_t>(kStepCount));

      AxisState direct_state = initial_state;
      std::vector<double> direct_history = initial_history;
      for (int step_index = 0;
        step_index < kStepCount; ++step_index)
      {
        ASSERT_TRUE(
          apply_projected_fir_step_in_place(
            direct_state, limits, coefficients, direct_history,
            input, kTimeStep));
        const AxisState & affine_state =
          affine_states[static_cast<std::size_t>(step_index)];
        EXPECT_NEAR(
          affine_state.velocity, direct_state.velocity, 1.0e-12);
        EXPECT_NEAR(
          affine_state.acceleration, direct_state.acceleration, 1.0e-12);
      }
    }
  }
  EXPECT_GE(feasible_trial_count, 90);
}

}  // namespace f_dwa_controller
