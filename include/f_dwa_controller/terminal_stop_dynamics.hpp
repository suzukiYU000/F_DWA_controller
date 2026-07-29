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

#ifndef F_DWA_CONTROLLER__TERMINAL_STOP_DYNAMICS_HPP_
#define F_DWA_CONTROLLER__TERMINAL_STOP_DYNAMICS_HPP_

#include <vector>

#include "f_dwa_controller/native_input_dynamics.hpp"

namespace f_dwa_controller
{

struct StopSequence
{
  std::vector<double> native_inputs;
  std::vector<AxisState> states;
  std::vector<std::vector<double>> fir_histories;
  bool feasible{false};
  bool terminal_state_cleared{false};
};

struct FirStopCoefficientResponse
{
  std::vector<double> coefficients;
  std::vector<double> held_unit_accelerations;
  std::vector<double> held_unit_velocities;
  double time_step{0.0};
  bool valid{false};
};

FirStopCoefficientResponse prepare_fir_stop_coefficient_response(
  const std::vector<double> & coefficients,
  double time_step);

StopSequence generate_acceleration_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  double time_step,
  int maximum_steps,
  double stop_velocity_threshold);

StopSequence generate_jerk_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  double time_step,
  int maximum_steps,
  double stop_velocity_threshold);

StopSequence generate_fir_stop_sequence(
  const AxisState & initial_state,
  const AxisLimits & limits,
  const std::vector<double> & coefficients,
  const std::vector<double> & history,
  double time_step,
  int maximum_steps,
  double stop_velocity_threshold,
  bool record_fir_histories = true,
  const FirStopCoefficientResponse * coefficient_response = nullptr);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TERMINAL_STOP_DYNAMICS_HPP_
