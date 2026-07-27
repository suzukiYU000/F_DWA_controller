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

#include "f_dwa_controller/trajectory_certifier.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace f_dwa_controller
{

namespace
{

CertificationResult check_pose(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const geometry_msgs::msg::Pose2D & pose)
{
  CertificationResult result;
  result.checked_pose_count = 1;
  std::vector<nav2_costmap_2d::MapLocation> map_footprint;
  map_footprint.reserve(footprint.size());
  const double cosine = std::cos(pose.theta);
  const double sine = std::sin(pose.theta);
  for (const geometry_msgs::msg::Point & point : footprint) {
    const double world_x =
      pose.x + point.x * cosine - point.y * sine;
    const double world_y =
      pose.y + point.x * sine + point.y * cosine;
    nav2_costmap_2d::MapLocation map_point;
    if (!costmap.worldToMap(
        world_x, world_y, map_point.x, map_point.y))
    {
      result.failure = CertificationFailure::kOffCostmap;
      return result;
    }
    map_footprint.push_back(map_point);
  }

  std::vector<nav2_costmap_2d::MapLocation> footprint_cells;
  costmap.convexFillCells(map_footprint, footprint_cells);
  for (const nav2_costmap_2d::MapLocation & cell : footprint_cells) {
    const unsigned char cost = costmap.getCost(cell.x, cell.y);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      result.failure = CertificationFailure::kUnknownSpace;
      return result;
    }
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      result.failure = CertificationFailure::kLethalObstacle;
      return result;
    }
  }
  result.safe = true;
  result.failure = CertificationFailure::kNone;
  return result;
}

double normalized_angle_difference(
  const double from,
  const double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

}  // namespace

CertificationResult certify_pose_sequence(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const double maximum_swept_distance)
{
  CertificationResult result;
  if (footprint.size() < 3u || poses.empty() ||
    !std::isfinite(maximum_swept_distance) ||
    maximum_swept_distance <= 0.0)
  {
    return result;
  }

  double maximum_footprint_radius = 0.0;
  for (const geometry_msgs::msg::Point & point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return result;
    }
    maximum_footprint_radius =
      std::max(maximum_footprint_radius, std::hypot(point.x, point.y));
  }

  CertificationResult pose_result =
    check_pose(costmap, footprint, poses.front());
  result.checked_pose_count += pose_result.checked_pose_count;
  if (!pose_result.safe) {
    result.failure = pose_result.failure;
    return result;
  }

  for (std::size_t pose_index = 1u;
    pose_index < poses.size(); ++pose_index)
  {
    const geometry_msgs::msg::Pose2D & previous = poses[pose_index - 1u];
    const geometry_msgs::msg::Pose2D & next = poses[pose_index];
    if (!std::isfinite(previous.x) || !std::isfinite(previous.y) ||
      !std::isfinite(previous.theta) || !std::isfinite(next.x) ||
      !std::isfinite(next.y) || !std::isfinite(next.theta))
    {
      result.failure = CertificationFailure::kInvalidInput;
      return result;
    }
    const double angle_difference =
      normalized_angle_difference(previous.theta, next.theta);
    const double swept_distance =
      std::hypot(next.x - previous.x, next.y - previous.y) +
      maximum_footprint_radius * std::abs(angle_difference);
    const int interpolation_count =
      std::max(
      1, static_cast<int>(
        std::ceil(swept_distance / maximum_swept_distance)));
    for (int interpolation_index = 1;
      interpolation_index <= interpolation_count; ++interpolation_index)
    {
      const double ratio =
        static_cast<double>(interpolation_index) /
        static_cast<double>(interpolation_count);
      geometry_msgs::msg::Pose2D interpolated;
      interpolated.x = previous.x + ratio * (next.x - previous.x);
      interpolated.y = previous.y + ratio * (next.y - previous.y);
      interpolated.theta = previous.theta + ratio * angle_difference;
      pose_result = check_pose(costmap, footprint, interpolated);
      result.checked_pose_count += pose_result.checked_pose_count;
      if (!pose_result.safe) {
        result.failure = pose_result.failure;
        return result;
      }
    }
  }

  result.safe = true;
  result.failure = CertificationFailure::kNone;
  return result;
}

const char * certification_failure_name(
  const CertificationFailure failure)
{
  switch (failure) {
    case CertificationFailure::kNone:
      return "none";
    case CertificationFailure::kInvalidInput:
      return "invalid_input";
    case CertificationFailure::kOffCostmap:
      return "off_costmap";
    case CertificationFailure::kLethalObstacle:
      return "lethal_obstacle";
    case CertificationFailure::kUnknownSpace:
      return "unknown_space";
  }
  return "invalid_input";
}

}  // namespace f_dwa_controller
