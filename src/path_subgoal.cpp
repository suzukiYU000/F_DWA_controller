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

#include "f_dwa_controller/path_subgoal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace f_dwa_controller
{

bool compute_path_subgoal(
  const nav_2d_msgs::msg::Path2D & path,
  const geometry_msgs::msg::Pose2D & current_pose,
  const double lookahead_distance,
  geometry_msgs::msg::Pose2D & subgoal)
{
  if (path.poses.empty() || !std::isfinite(lookahead_distance) ||
    lookahead_distance < 0.0)
  {
    return false;
  }
  if (path.poses.size() == 1u) {
    subgoal = path.poses.front();
    return true;
  }

  std::vector<double> cumulative_distance(path.poses.size(), 0.0);
  double nearest_squared_distance = std::numeric_limits<double>::infinity();
  double current_progress = 0.0;
  bool has_nonzero_segment = false;
  for (std::size_t index = 1u; index < path.poses.size(); ++index) {
    const auto & start = path.poses[index - 1u];
    const auto & end = path.poses[index];
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_squared_length =
      segment_x * segment_x + segment_y * segment_y;
    const double segment_length = std::sqrt(segment_squared_length);
    cumulative_distance[index] =
      cumulative_distance[index - 1u] + segment_length;
    if (segment_squared_length <= 1.0e-12) {
      continue;
    }
    has_nonzero_segment = true;
    const double projection = std::clamp(
      ((current_pose.x - start.x) * segment_x +
      (current_pose.y - start.y) * segment_y) / segment_squared_length,
      0.0, 1.0);
    const double projected_x = start.x + projection * segment_x;
    const double projected_y = start.y + projection * segment_y;
    const double squared_distance =
      (current_pose.x - projected_x) * (current_pose.x - projected_x) +
      (current_pose.y - projected_y) * (current_pose.y - projected_y);
    if (squared_distance < nearest_squared_distance) {
      nearest_squared_distance = squared_distance;
      current_progress = cumulative_distance[index - 1u] +
        projection * segment_length;
    }
  }
  if (!has_nonzero_segment) {
    subgoal = path.poses.back();
    return true;
  }

  const double target_progress = std::min(
    cumulative_distance.back(), current_progress + lookahead_distance);
  for (std::size_t index = 1u; index < path.poses.size(); ++index) {
    if (target_progress > cumulative_distance[index]) {
      continue;
    }
    const auto & start = path.poses[index - 1u];
    const auto & end = path.poses[index];
    const double segment_length =
      cumulative_distance[index] - cumulative_distance[index - 1u];
    if (segment_length <= 1.0e-12) {
      continue;
    }
    const double interpolation = std::clamp(
      (target_progress - cumulative_distance[index - 1u]) /
      segment_length,
      0.0, 1.0);
    subgoal.x = start.x + interpolation * (end.x - start.x);
    subgoal.y = start.y + interpolation * (end.y - start.y);
    subgoal.theta = std::atan2(end.y - start.y, end.x - start.x);
    return true;
  }

  subgoal = path.poses.back();
  return true;
}

double path_subgoal_progress_cost(
  const geometry_msgs::msg::Pose2D & terminal_pose,
  const geometry_msgs::msg::Pose2D & subgoal)
{
  const double longitudinal_lack =
    (subgoal.x - terminal_pose.x) * std::cos(subgoal.theta) +
    (subgoal.y - terminal_pose.y) * std::sin(subgoal.theta);
  return std::max(0.0, longitudinal_lack);
}

double path_subgoal_forward_ray_cost(
  const geometry_msgs::msg::Pose2D & terminal_pose,
  const geometry_msgs::msg::Pose2D & subgoal,
  const double lateral_weight)
{
  const double tangent_x = std::cos(subgoal.theta);
  const double tangent_y = std::sin(subgoal.theta);
  const double subgoal_to_terminal_x = terminal_pose.x - subgoal.x;
  const double subgoal_to_terminal_y = terminal_pose.y - subgoal.y;
  const double longitudinal_progress =
    subgoal_to_terminal_x * tangent_x +
    subgoal_to_terminal_y * tangent_y;
  const double lateral_error =
    -subgoal_to_terminal_x * tangent_y +
    subgoal_to_terminal_y * tangent_x;
  return std::hypot(
    std::max(0.0, -longitudinal_progress), lateral_weight * lateral_error);
}

}  // namespace f_dwa_controller
