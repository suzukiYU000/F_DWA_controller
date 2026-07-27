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

#ifndef F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_
#define F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace f_dwa_controller
{

enum class CertificationFailure
{
  kNone,
  kInvalidInput,
  kOffCostmap,
  kLethalObstacle,
  kUnknownSpace
};

struct CertificationResult
{
  bool safe{false};
  CertificationFailure failure{CertificationFailure::kInvalidInput};
  std::size_t checked_pose_count{0};
};

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  double maximum_swept_distance);

const char * certification_failure_name(CertificationFailure failure);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__TRAJECTORY_CERTIFIER_HPP_
