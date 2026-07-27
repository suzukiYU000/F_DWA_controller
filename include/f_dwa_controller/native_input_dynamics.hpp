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

#ifndef F_DWA_CONTROLLER__NATIVE_INPUT_DYNAMICS_HPP_
#define F_DWA_CONTROLLER__NATIVE_INPUT_DYNAMICS_HPP_

#include <vector>

namespace f_dwa_controller
{

struct FeasibleInterval
{
  double lower{0.0};
  double upper{0.0};
  bool feasible{false};
};

struct AxisState
{
  double velocity{0.0};
  double acceleration{0.0};
};

struct AxisLimits
{
  double velocity_min{0.0};
  double velocity_max{0.0};
  double acceleration_min{0.0};
  double acceleration_max{0.0};
  double native_input_min{0.0};
  double native_input_max{0.0};
};

struct ProjectedAxisStep
{
  AxisState state;
  double applied_native_input{0.0};
  bool feasible{false};
};

FeasibleInterval acceleration_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  double time_step);

FeasibleInterval jerk_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  double time_step);

FeasibleInterval held_acceleration_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  double time_step,
  int remaining_steps);

FeasibleInterval held_jerk_input_interval(
  const AxisState & state,
  const AxisLimits & limits,
  double time_step,
  int remaining_steps);

ProjectedAxisStep project_acceleration_step(
  const AxisState & state,
  const AxisLimits & limits,
  double native_input_reference,
  double time_step);

ProjectedAxisStep project_jerk_step(
  const AxisState & state,
  const AxisLimits & limits,
  double native_input_reference,
  double time_step);

ProjectedAxisStep project_held_acceleration_step(
  const AxisState & state,
  const AxisLimits & limits,
  double native_input_reference,
  double time_step,
  int remaining_steps);

ProjectedAxisStep project_held_jerk_step(
  const AxisState & state,
  const AxisLimits & limits,
  double native_input_reference,
  double time_step,
  int remaining_steps);

std::vector<double> uniform_samples(
  const FeasibleInterval & interval,
  int sample_count);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__NATIVE_INPUT_DYNAMICS_HPP_
