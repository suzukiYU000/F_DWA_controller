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

#ifndef F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_
#define F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_

#include "dwb_plugins/limited_accel_generator.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"

namespace f_dwa_controller
{

// These adapters intentionally add no behavior. Exporting the Nav2 generators
// from the same library as CertifiedDWBLocalPlanner avoids loading their
// factories through a second class loader after this library has already linked
// them as dependencies.
class VLimitedAccelTrajectoryGenerator
  : public dwb_plugins::LimitedAccelGenerator
{
};

class VStandardTrajectoryGenerator
  : public dwb_plugins::StandardTrajectoryGenerator
{
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__V_DWB_TRAJECTORY_GENERATORS_HPP_
