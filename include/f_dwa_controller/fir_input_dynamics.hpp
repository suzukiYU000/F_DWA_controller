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

#ifndef F_DWA_CONTROLLER__FIR_INPUT_DYNAMICS_HPP_
#define F_DWA_CONTROLLER__FIR_INPUT_DYNAMICS_HPP_

#include <vector>

#include "f_dwa_controller/native_input_dynamics.hpp"

namespace f_dwa_controller
{

struct ProjectedFirStep
{
  AxisState state;
  std::vector<double> history;
  double applied_native_input{0.0};
  bool feasible{false};
};

struct HeldFirAffineResponse
{
  FeasibleInterval input_interval;
  std::vector<AxisState> free_states;
  std::vector<AxisState> unit_states;
};

double fir_acceleration(
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double native_input);

void push_fir_input(
  std::vector<double> & history,
  double native_input);

HeldFirAffineResponse prepare_held_fir_affine_response(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double time_step,
  int remaining_steps);

// Builds an affine rollout in which the sampled raw input is active for
// active_input_steps and is then zero. The full remaining horizon is still
// checked against the acceleration and velocity limits.
HeldFirAffineResponse prepare_pulsed_fir_affine_response(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double time_step,
  int remaining_steps,
  int active_input_steps);

bool sample_held_fir_affine_response(
  const HeldFirAffineResponse & response,
  const AxisLimits & limits,
  double held_native_input,
  std::vector<AxisState> & states);

FeasibleInterval held_fir_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double time_step,
  int remaining_steps);

ProjectedFirStep project_held_fir_step(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double native_input_reference,
  double time_step,
  int remaining_steps);

// Applies an input that was already projected against the full-horizon
// feasible interval. It never clamps the input; a constraint violation makes
// the step infeasible so the model cannot silently diverge from the command.
ProjectedFirStep apply_projected_fir_step(
  const AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double projected_native_input,
  double time_step);

// Hot-loop variant of apply_projected_fir_step. The FIR history is updated only
// after a feasible step, avoiding two history copies per rollout sample.
bool apply_projected_fir_step_in_place(
  AxisState & state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  std::vector<double> & history,
  double projected_native_input,
  double time_step);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FIR_INPUT_DYNAMICS_HPP_
