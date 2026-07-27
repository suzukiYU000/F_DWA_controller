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

#ifndef F_DWA_CONTROLLER__DWA_CONTROLLERS_HPP_
#define F_DWA_CONTROLLER__DWA_CONTROLLERS_HPP_

#include "f_dwa_controller/certified_dwb_local_planner.hpp"

namespace f_dwa_controller
{

class ADwaController final : public CertifiedDWBLocalPlanner
{
public:
  ADwaController() = default;
  ~ADwaController() override = default;
};

class JDwaController final : public CertifiedDWBLocalPlanner
{
public:
  JDwaController() = default;
  ~JDwaController() override = default;
};

class FDwaController final : public CertifiedDWBLocalPlanner
{
public:
  FDwaController() = default;
  ~FDwaController() override = default;
};

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__DWA_CONTROLLERS_HPP_
