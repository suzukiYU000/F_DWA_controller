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

#ifndef F_DWA_CONTROLLER__PATH_SUBGOAL_HPP_
#define F_DWA_CONTROLLER__PATH_SUBGOAL_HPP_

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace f_dwa_controller
{

bool compute_path_subgoal(
  const nav_2d_msgs::msg::Path2D & path,
  const geometry_msgs::msg::Pose2D & current_pose,
  double lookahead_distance,
  geometry_msgs::msg::Pose2D & subgoal);

double path_subgoal_progress_cost(
  const geometry_msgs::msg::Pose2D & terminal_pose,
  const geometry_msgs::msg::Pose2D & subgoal);

double path_subgoal_forward_ray_cost(
  const geometry_msgs::msg::Pose2D & terminal_pose,
  const geometry_msgs::msg::Pose2D & subgoal,
  double lateral_weight = 1.0);

}  // namespace f_dwa_controller

#endif  // F_DWA_CONTROLLER__PATH_SUBGOAL_HPP_
