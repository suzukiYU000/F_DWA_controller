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

double fir_acceleration(
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double native_input);

void push_fir_input(
  std::vector<double> & history,
  double native_input);

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

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__FIR_INPUT_DYNAMICS_HPP_
