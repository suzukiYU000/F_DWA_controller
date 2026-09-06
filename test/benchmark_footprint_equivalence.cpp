// Copyright 2026 YT Lab
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "f_dwa_controller/horizon_obstacle_footprint_critic.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace
{
using namespace f_dwa_controller;

std::vector<geometry_msgs::msg::Point> footprint()
{
  std::vector<geometry_msgs::msg::Point> points(4);
  points[0].x = -0.2; points[0].y = -0.3;
  points[1].x = 0.8; points[1].y = -0.3;
  points[2].x = 0.8; points[2].y = 0.3;
  points[3].x = -0.2; points[3].y = 0.3;
  return points;
}

class Critic : public HorizonObstacleFootprintCritic
{
public:
  void prepareSnapshot(nav2_costmap_2d::Costmap2D & map, CertificationWorkspace & workspace)
  {
    costmap_ = &map;
    name_ = "ObstacleFootprint";
    inset_core_footprint_ = footprint();
    footprint_spec_ = inset_core_footprint_;
    nav2_costmap_2d::padFootprint(footprint_spec_, 0.05);
    footprint_radius_ = std::hypot(0.85, 0.35);
    enable_initial_overlap_recovery_ = true;
    enable_transient_boundary_margin_recovery_ = true;
    shared_certification_workspace_ = &workspace;
    raster_footprint_cache_.clear();
    cell_diagnostic_cache_.clear();
    raster_footprint_cache_ready_ = true;
    certification_workspace_prepared_ = false;
  }
};

void print_certificate(const CertificationResult & result)
{
  std::cout << result.safe << ',' << static_cast<int>(result.failure) << ',' <<
    result.checked_pose_count << ',' << result.has_failure_pose << ',' <<
    result.failure_source_pose_index << ',' << result.failure_interpolation_index << ',' <<
    result.failure_pose.x << ',' << result.failure_pose.y << ',' << result.failure_pose.theta <<
    ',' <<
    result.has_failure_cell << ',' << result.failure_cell_x << ',' << result.failure_cell_y <<
    ',' <<
    static_cast<int>(result.failure_cell_cost) << ',' <<
    result.failure_cell_world_x << ',' << result.failure_cell_world_y;
}
}  // namespace

int main()
{
  // Pure Costmap/vector computations; no ROS node or command transport exists.
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "94") {
    return 2;
  }
  std::cout << std::hexfloat;
  const auto body = footprint();
  auto planning = body;
  nav2_costmap_2d::padFootprint(planning, 0.05);
  std::size_t case_id = 0;
  for (int width_index = 0; width_index <= 26; ++width_index) {
    const double width = 0.45 + 0.05 * width_index;
    for (int phase = 0; phase < 4; ++phase) {
      nav2_costmap_2d::Costmap2D map(180, 140, 0.025, -0.75, -1.75 + phase * 0.00625, 0);
      for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
        for (unsigned int x = 0; x < map.getSizeInCellsX(); ++x) {
          double wx, wy;
          map.mapToWorld(x, y, wx, wy);
          if (std::abs(wy) >= width * 0.5 ||
            (phase == 3 && wx > 1.5 && wx < 1.7 && wy > width * 0.5 - 0.15))
          {
            map.setCost(x, y, phase == 2 ? 255 : 254);
          }
        }
      }
      CertificationWorkspace workspace;
      if (!prepare_certification_broadphase(map, workspace)) {return 3;}
      Critic critic;
      critic.prepareSnapshot(map, workspace);
      for (const double offset : {-0.15, 0.0, 0.15}) {
        for (const double heading : {-0.3, -0.15, 0.0, 0.15, 0.3}) {
          for (const double turn : {-0.8, -0.3, 0.0, 0.3, 0.8}) {
            dwb_msgs::msg::Trajectory2D trajectory;
            geometry_msgs::msg::Pose2D pose;
            pose.y = offset; pose.theta = heading;
            for (int step = 0; step <= 50; ++step) {
              trajectory.poses.push_back(pose);
              pose.x += 0.04 * std::cos(pose.theta);
              pose.y += 0.04 * std::sin(pose.theta);
              pose.theta += 0.05 * turn;
            }
            std::cout << case_id++ << '\t';
            try {
              std::cout << critic.scoreTrajectory(trajectory);
            } catch (const dwb_core::IllegalTrajectoryException & error) {
              std::cout << error.what();
            }
            std::cout << '\t';
            print_certificate(certify_pose_sequence(map, body, trajectory.poses, 0.025,
              &workspace));
            for (const bool depth : {false, true}) {
              double overlap = 0.0;
              CertificationResult physical;
              const bool safe = certify_initial_overlap_margin_sequence(map, planning, body,
                trajectory.poses, 0.025, &overlap, &workspace, true, true, 0.5, 0.1, depth,
                &physical);
              std::cout << '\t' << safe << ',' << overlap << ',';
              print_certificate(physical);
            }
            std::cout << '\n';
          }
        }
      }
    }
  }
}
