/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2026, Keio University
 * All rights reserved.
 */

#include "f_dwa_controller/fixed_distance_risk_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr double kDistanceTolerance = 1.0e-9;
constexpr double kCurvatureTolerance = 1.0e-9;
constexpr double kTimeTolerance = 1.0e-9;
// A heading inferred from less than four 0.025 m Costmap cells is dominated by
// discretization and in-place yaw. Requiring this common translation chord
// prevents a stopped or very-low-speed/high-yaw candidate from being credited
// with a fictional metre-long lateral bypass.
constexpr double kMinimumCredibleHeadingTranslation = 0.10;
// The 2.5 m common risk horizon leaves about 1.66 m beyond a maximum-speed
// 0.84 m seed. Holding the spatial candidate heading for the first 0.50 m,
// then using the configurable 1.0 m relaxation, lets a genuine avoidance
// maneuver become distinguishable while still rejoining the plan within that
// common horizon. This is a soft observation only and is identical for every
// V/A/J/F candidate.
constexpr double kManeuverHeadingHoldDistance = 0.50;
constexpr std::size_t kMaximumRiskPathSamples = 100000u;

void validate_plan_continuation_distances(
  const double risk_distance, const double sample_resolution,
  const double heading_relaxation_distance)
{
  if (!std::isfinite(risk_distance) || risk_distance <= 0.0 ||
    !std::isfinite(sample_resolution) || sample_resolution <= 0.0 ||
    !std::isfinite(heading_relaxation_distance) ||
    heading_relaxation_distance <= 0.0)
  {
    throw std::invalid_argument{
            "plan-continuation risk path requires positive finite distances"};
  }
}

bool is_finite_pose(const geometry_msgs::msg::Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.theta);
}

double interpolate_angle(
  const double first, const double second, const double ratio)
{
  return first + ratio * std::remainder(second - first, 2.0 * M_PI);
}

double estimate_terminal_curvature(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const std::vector<double> & cumulative_distance,
  const double estimation_distance)
{
  double measured_distance = 0.0;
  double measured_yaw_change = 0.0;
  for (std::size_t index = trajectory.poses.size(); index > 1u; --index) {
    const std::size_t upper_index = index - 1u;
    const std::size_t lower_index = upper_index - 1u;
    const double segment_length =
      cumulative_distance[upper_index] - cumulative_distance[lower_index];
    if (segment_length <= kDistanceTolerance) {
      continue;
    }
    measured_distance += segment_length;
    measured_yaw_change += std::remainder(
      trajectory.poses[upper_index].theta -
      trajectory.poses[lower_index].theta,
      2.0 * M_PI);
    if (measured_distance + kDistanceTolerance >= estimation_distance) {
      break;
    }
  }
  if (measured_distance <= kDistanceTolerance) {
    return 0.0;
  }
  const double curvature = measured_yaw_change / measured_distance;
  if (!std::isfinite(curvature)) {
    throw std::invalid_argument{
            "fixed-distance risk path curvature is non-finite"};
  }
  return curvature;
}

geometry_msgs::msg::Pose2D extend_pose(
  const geometry_msgs::msg::Pose2D & endpoint,
  const double extension,
  const double curvature)
{
  geometry_msgs::msg::Pose2D pose = endpoint;
  if (std::abs(curvature) <= kCurvatureTolerance) {
    pose.x += extension * std::cos(endpoint.theta);
    pose.y += extension * std::sin(endpoint.theta);
    return pose;
  }

  const double yaw_change = curvature * extension;
  if (!std::isfinite(yaw_change)) {
    throw std::invalid_argument{
            "fixed-distance risk path extension is non-finite"};
  }
  const double wrapped_yaw_change = std::remainder(
    yaw_change, 2.0 * M_PI);
  const double extended_heading = endpoint.theta + wrapped_yaw_change;
  pose.x += (std::sin(extended_heading) - std::sin(endpoint.theta)) /
    curvature;
  pose.y += (std::cos(endpoint.theta) - std::cos(extended_heading)) /
    curvature;
  pose.theta = extended_heading;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.theta))
  {
    throw std::invalid_argument{
            "fixed-distance risk path extension produced a non-finite pose"};
  }
  return pose;
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         1.0e-9 * static_cast<double>(duration.nanosec);
}

void trajectory_prefix_at_time(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double risk_seed_time,
  const std::size_t additional_pose_capacity,
  dwb_msgs::msg::Trajectory2D & prefix)
{
  if (!std::isfinite(risk_seed_time) || risk_seed_time < 0.0) {
    throw std::invalid_argument{
            "fixed-distance risk seed time must be finite and non-negative"};
  }
  if (trajectory.poses.empty() || trajectory.time_offsets.empty()) {
    throw std::invalid_argument{
            "fixed-distance timed prefix requires poses and time offsets"};
  }

  const std::size_t timed_pose_count = trajectory.time_offsets.size();
  if (trajectory.poses.size() != timed_pose_count &&
    trajectory.poses.size() != timed_pose_count + 1u)
  {
    throw std::invalid_argument{
            "fixed-distance timed prefix has an unsupported Nav2 layout"};
  }

  double first_pose_time = 0.0;
  double previous_pose_time = 0.0;
  for (std::size_t index = 0u; index < timed_pose_count; ++index) {
    const double pose_time = duration_seconds(trajectory.time_offsets[index]);
    if (!std::isfinite(pose_time) || pose_time < -kTimeTolerance ||
      (index > 0u && pose_time <= previous_pose_time + kTimeTolerance))
    {
      throw std::invalid_argument{
              "fixed-distance timed prefix requires finite increasing offsets"};
    }
    if (index == 0u) {
      first_pose_time = pose_time;
    }
    previous_pose_time = pose_time;
  }
  if (std::abs(first_pose_time) > kTimeTolerance) {
    throw std::invalid_argument{
            "fixed-distance timed prefix must start at zero seconds"};
  }
  if (risk_seed_time > previous_pose_time + kTimeTolerance) {
    throw std::invalid_argument{
            "trajectory does not reach fixed-distance risk seed time"};
  }

  prefix.velocity = trajectory.velocity;
  prefix.time_offsets.clear();
  prefix.poses.clear();
  if (timed_pose_count > prefix.poses.max_size() ||
    additional_pose_capacity >
    prefix.poses.max_size() - timed_pose_count)
  {
    throw std::invalid_argument{
            "fixed-distance timed prefix requests too much capacity"};
  }
  prefix.poses.reserve(timed_pose_count + additional_pose_capacity);
  for (std::size_t index = 0u; index < timed_pose_count; ++index) {
    const double pose_time = duration_seconds(trajectory.time_offsets[index]);
    if (pose_time < risk_seed_time - kTimeTolerance) {
      prefix.poses.push_back(trajectory.poses[index]);
      continue;
    }
    if (std::abs(pose_time - risk_seed_time) <= kTimeTolerance) {
      prefix.poses.push_back(trajectory.poses[index]);
      return;
    }
    if (index == 0u || prefix.poses.empty()) {
      throw std::invalid_argument{
              "fixed-distance risk seed precedes the first timed pose"};
    }

    const double preceding_pose_time =
      duration_seconds(trajectory.time_offsets[index - 1u]);
    const double interval = pose_time - preceding_pose_time;
    const double ratio = std::clamp(
      (risk_seed_time - preceding_pose_time) / interval, 0.0, 1.0);
    const auto & first = trajectory.poses[index - 1u];
    const auto & second = trajectory.poses[index];
    geometry_msgs::msg::Pose2D interpolated;
    interpolated.x = first.x + ratio * (second.x - first.x);
    interpolated.y = first.y + ratio * (second.y - first.y);
    interpolated.theta = interpolate_angle(first.theta, second.theta, ratio);
    prefix.poses.push_back(interpolated);
    return;
  }

  throw std::invalid_argument{
          "trajectory does not contain the fixed-distance risk seed pose"};
}

dwb_msgs::msg::Trajectory2D trajectory_prefix_at_time(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double risk_seed_time,
  const std::size_t additional_pose_capacity = 0u)
{
  dwb_msgs::msg::Trajectory2D prefix;
  trajectory_prefix_at_time(
    trajectory, risk_seed_time, additional_pose_capacity, prefix);
  return prefix;
}

bool append_if_spatially_distinct(
  dwb_msgs::msg::Trajectory2D & trajectory,
  const geometry_msgs::msg::Pose2D & pose)
{
  if (!is_finite_pose(pose)) {
    throw std::invalid_argument{
            "plan continuation produced a non-finite pose"};
  }
  const auto & previous = trajectory.poses.back();
  if (std::hypot(pose.x - previous.x, pose.y - previous.y) >
    kDistanceTolerance)
  {
    trajectory.poses.push_back(pose);
    return true;
  }
  return false;
}

bool credible_spatial_heading(
  const dwb_msgs::msg::Trajectory2D & prefix, double & heading)
{
  const auto & endpoint = prefix.poses.back();
  for (std::size_t index = prefix.poses.size() - 1u; index > 0u; --index) {
    const auto & earlier = prefix.poses[index - 1u];
    const double delta_x = endpoint.x - earlier.x;
    const double delta_y = endpoint.y - earlier.y;
    const double displacement = std::hypot(delta_x, delta_y);
    if (!std::isfinite(displacement)) {
      throw std::invalid_argument{
              "plan continuation contains a non-finite candidate prefix"};
    }
    if (displacement + kDistanceTolerance >=
      kMinimumCredibleHeadingTranslation)
    {
      heading = std::atan2(delta_y, delta_x);
      return std::isfinite(heading);
    }
  }
  return false;
}

double relaxed_heading_error(
  const double initial_error, const double continuation_distance,
  const double relaxation_distance)
{
  if (continuation_distance <= kManeuverHeadingHoldDistance) {
    return initial_error;
  }
  const double relaxation_progress =
    continuation_distance - kManeuverHeadingHoldDistance;
  if (relaxation_progress >= relaxation_distance) {
    return 0.0;
  }
  return initial_error * std::max(
    0.0, 1.0 - relaxation_progress / relaxation_distance);
}

double relaxed_lateral_growth(
  const double initial_error, const double continuation_distance,
  const double relaxation_distance)
{
  const double hold_distance = std::clamp(
    continuation_distance, 0.0, kManeuverHeadingHoldDistance);
  const double hold_growth = hold_distance * std::sin(initial_error);
  if (continuation_distance <= kManeuverHeadingHoldDistance) {
    return hold_growth;
  }
  const double bounded_distance = std::clamp(
    continuation_distance - kManeuverHeadingHoldDistance,
    0.0, relaxation_distance);
  if (std::abs(initial_error) <= 1.0e-8) {
    // First-order limit of the closed-form integral of sin(e(s)). This avoids
    // subtracting two nearly equal cosines as e0 approaches zero.
    return hold_growth + initial_error * bounded_distance *
           (1.0 - 0.5 * bounded_distance / relaxation_distance);
  }
  const double remaining_fraction =
    1.0 - bounded_distance / relaxation_distance;
  // cos(a) - cos(b) in product form avoids cancellation for small but still
  // representable heading errors.
  return hold_growth - 2.0 * relaxation_distance / initial_error *
         std::sin(0.5 * initial_error * (remaining_fraction + 1.0)) *
         std::sin(0.5 * initial_error * (remaining_fraction - 1.0));
}

}  // namespace

PreparedPlanGeometry prepare_plan_continuation_geometry(
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  if (global_plan.poses.empty()) {
    throw std::invalid_argument{
            "plan-continuation risk path requires a global plan"};
  }
  for (const auto & pose : global_plan.poses) {
    if (!is_finite_pose(pose)) {
      throw std::invalid_argument{
              "plan-continuation risk path requires finite plan poses"};
    }
  }

  PreparedPlanGeometry geometry;
  geometry.poses_ = global_plan.poses;
  geometry.segments_.reserve(geometry.poses_.size() - 1u);
  double cumulative_progress = 0.0;
  for (std::size_t index = 1u; index < geometry.poses_.size(); ++index) {
    const auto & first = geometry.poses_[index - 1u];
    const auto & second = geometry.poses_[index];
    PreparedPlanGeometry::Segment segment;
    segment.delta_x = second.x - first.x;
    segment.delta_y = second.y - first.y;
    segment.length = std::hypot(segment.delta_x, segment.delta_y);
    segment.start_progress = cumulative_progress;
    if (!std::isfinite(segment.length)) {
      throw std::invalid_argument{
              "plan-continuation risk path has a non-finite segment"};
    }
    if (segment.length > kDistanceTolerance) {
      segment.heading = std::atan2(segment.delta_y, segment.delta_x);
      segment.heading_sine = std::sin(segment.heading);
      segment.heading_cosine = std::cos(segment.heading);
      geometry.has_plan_motion_ = true;
      geometry.last_moving_segment_ = index - 1u;
      cumulative_progress += segment.length;
      if (!std::isfinite(cumulative_progress)) {
        throw std::invalid_argument{
                "plan-continuation risk path length overflowed"};
      }
    }
    geometry.segments_.push_back(segment);
  }

  // One forward and one reverse pass reproduce plan_vertex_heading() without
  // scanning over duplicate vertices for every candidate and vertex.
  geometry.vertex_headings_.resize(
    geometry.poses_.size(), std::numeric_limits<double>::quiet_NaN());
  double previous_heading = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t vertex = 0u; vertex < geometry.poses_.size(); ++vertex) {
    if (vertex > 0u &&
      geometry.segments_[vertex - 1u].length > kDistanceTolerance)
    {
      previous_heading = geometry.segments_[vertex - 1u].heading;
    }
    geometry.vertex_headings_[vertex] = previous_heading;
  }
  double next_heading = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t vertex = geometry.poses_.size(); vertex-- > 0u; ) {
    if (vertex < geometry.segments_.size() &&
      geometry.segments_[vertex].length > kDistanceTolerance)
    {
      next_heading = geometry.segments_[vertex].heading;
    }
    const double incoming_heading = geometry.vertex_headings_[vertex];
    if (std::isfinite(incoming_heading) && std::isfinite(next_heading)) {
      geometry.vertex_headings_[vertex] = interpolate_angle(
        incoming_heading, next_heading, 0.5);
    } else if (std::isfinite(next_heading)) {
      geometry.vertex_headings_[vertex] = next_heading;
    } else if (!std::isfinite(incoming_heading)) {
      geometry.vertex_headings_[vertex] = geometry.poses_[vertex].theta;
    }
  }
  return geometry;
}

std::vector<RiskPathSample> build_fixed_distance_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double risk_distance,
  const double sample_resolution)
{
  if (!std::isfinite(risk_distance) || risk_distance <= 0.0 ||
    !std::isfinite(sample_resolution) || sample_resolution <= 0.0)
  {
    throw std::invalid_argument{
            "fixed-distance risk path requires positive finite distances"};
  }
  if (trajectory.poses.empty()) {
    throw std::invalid_argument{
            "fixed-distance risk path requires at least one pose"};
  }
  for (const auto & pose : trajectory.poses) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.theta))
    {
      throw std::invalid_argument{
              "fixed-distance risk path requires finite poses"};
    }
  }

  std::vector<double> cumulative_distance(trajectory.poses.size(), 0.0);
  for (std::size_t index = 1u; index < trajectory.poses.size(); ++index) {
    const auto & previous = trajectory.poses[index - 1u];
    const auto & current = trajectory.poses[index];
    const double segment_length = std::hypot(
      current.x - previous.x, current.y - previous.y);
    if (!std::isfinite(segment_length)) {
      throw std::invalid_argument{
              "fixed-distance risk path contains a non-finite segment"};
    }
    cumulative_distance[index] =
      cumulative_distance[index - 1u] + segment_length;
    if (!std::isfinite(cumulative_distance[index])) {
      throw std::invalid_argument{
              "fixed-distance risk path length overflowed"};
    }
  }
  const double nominal_distance = cumulative_distance.back();
  const bool has_spatial_motion = nominal_distance > kDistanceTolerance;
  const auto & endpoint = has_spatial_motion ?
    trajectory.poses.back() : trajectory.poses.front();
  const double terminal_curvature = has_spatial_motion ?
    estimate_terminal_curvature(
    trajectory, cumulative_distance, sample_resolution) : 0.0;

  std::vector<double> target_distances;
  const double complete_steps_value =
    std::floor(risk_distance / sample_resolution + kDistanceTolerance);
  if (!std::isfinite(complete_steps_value) || complete_steps_value < 0.0 ||
    complete_steps_value >
    static_cast<double>(kMaximumRiskPathSamples - 2u))
  {
    throw std::invalid_argument{
            "fixed-distance risk path requests too many samples"};
  }
  const std::size_t complete_steps =
    static_cast<std::size_t>(complete_steps_value);
  target_distances.reserve(complete_steps + 2u);
  for (std::size_t index = 0u; index <= complete_steps; ++index) {
    target_distances.push_back(std::min(
      risk_distance, sample_resolution * static_cast<double>(index)));
  }
  if (target_distances.empty() ||
    risk_distance - target_distances.back() > kDistanceTolerance)
  {
    target_distances.push_back(risk_distance);
  }

  std::vector<RiskPathSample> samples;
  samples.reserve(target_distances.size());
  std::size_t upper_index = 1u;
  for (const double target : target_distances) {
    RiskPathSample sample;
    sample.arc_length = target;
    const bool target_is_inside_nominal_path =
      target < nominal_distance && trajectory.poses.size() > 1u &&
      nominal_distance > kDistanceTolerance;
    if (target <= kDistanceTolerance) {
      sample.pose = trajectory.poses.front();
    } else if (std::abs(target - nominal_distance) <= kDistanceTolerance) {
      // Preserve trailing in-place rotation and make the extension orientation
      // exactly continuous with the final nominal pose.
      sample.pose = endpoint;
    } else if (target_is_inside_nominal_path) {
      while (upper_index < cumulative_distance.size() &&
        cumulative_distance[upper_index] < target - kDistanceTolerance)
      {
        ++upper_index;
      }
      upper_index = std::min(
        upper_index, cumulative_distance.size() - 1u);
      std::size_t lower_index = upper_index - 1u;
      while (upper_index < cumulative_distance.size() - 1u &&
        cumulative_distance[upper_index] -
        cumulative_distance[lower_index] <= kDistanceTolerance)
      {
        ++upper_index;
      }
      lower_index = upper_index - 1u;
      const double segment_length =
        cumulative_distance[upper_index] - cumulative_distance[lower_index];
      const double ratio = segment_length > kDistanceTolerance ?
        std::clamp(
        (target - cumulative_distance[lower_index]) / segment_length,
        0.0, 1.0) : 1.0;
      const auto & first = trajectory.poses[lower_index];
      const auto & second = trajectory.poses[upper_index];
      sample.pose.x = first.x + ratio * (second.x - first.x);
      sample.pose.y = first.y + ratio * (second.y - first.y);
      sample.pose.theta = interpolate_angle(first.theta, second.theta, ratio);
    } else {
      const double extension = std::max(0.0, target - nominal_distance);
      sample.pose = extend_pose(
        endpoint, extension, terminal_curvature);
    }
    samples.push_back(std::move(sample));
  }
  return samples;
}

std::vector<RiskPathSample> build_fixed_distance_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double risk_distance,
  const double sample_resolution,
  const double risk_seed_time)
{
  return build_fixed_distance_risk_path(
    trajectory_prefix_at_time(trajectory, risk_seed_time),
    risk_distance, sample_resolution);
}

std::vector<RiskPathSample> build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const nav_2d_msgs::msg::Path2D & global_plan,
  const double risk_distance,
  const double sample_resolution,
  const double risk_seed_time,
  const double heading_relaxation_distance)
{
  // Keep the validation order of the compatibility API: invalid distances
  // are reported before plan preparation.
  validate_plan_continuation_distances(
    risk_distance, sample_resolution, heading_relaxation_distance);
  return build_plan_continued_risk_path(
    trajectory, prepare_plan_continuation_geometry(global_plan),
    risk_distance, sample_resolution, risk_seed_time,
    heading_relaxation_distance);
}

std::vector<RiskPathSample> build_plan_continued_risk_path(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const PreparedPlanGeometry & plan_geometry,
  const double risk_distance,
  const double sample_resolution,
  const double risk_seed_time,
  const double heading_relaxation_distance)
{
  validate_plan_continuation_distances(
    risk_distance, sample_resolution, heading_relaxation_distance);
  if (plan_geometry.empty()) {
    throw std::invalid_argument{
            "plan-continuation risk path requires a global plan"};
  }

  dwb_msgs::msg::Trajectory2D combined =
    trajectory_prefix_at_time(
    trajectory, risk_seed_time, plan_geometry.poses_.size());
  const auto endpoint = combined.poses.back();
  double combined_distance = 0.0;
  for (std::size_t index = 1u; index < combined.poses.size(); ++index) {
    const double segment_length = std::hypot(
      combined.poses[index].x - combined.poses[index - 1u].x,
      combined.poses[index].y - combined.poses[index - 1u].y);
    combined_distance += segment_length;
    if (!std::isfinite(combined_distance)) {
      throw std::invalid_argument{
              "plan continuation contains a non-finite candidate prefix"};
    }
  }
  if (combined_distance + kDistanceTolerance >= risk_distance) {
    return build_fixed_distance_risk_path(
      combined, risk_distance, sample_resolution);
  }

  if (!plan_geometry.has_plan_motion_) {
    const double heading = plan_geometry.poses_.front().theta;
    geometry_msgs::msg::Pose2D terminal = endpoint;
    terminal.x += risk_distance * std::cos(heading);
    terminal.y += risk_distance * std::sin(heading);
    append_if_spatially_distinct(combined, terminal);
    return build_fixed_distance_risk_path(
      combined, risk_distance, sample_resolution);
  }

  double projection_progress = 0.0;
  double nearest_squared_distance = std::numeric_limits<double>::infinity();
  std::size_t projection_segment = 0u;
  for (std::size_t index = 0u;
    index < plan_geometry.segments_.size(); ++index)
  {
    const auto & segment = plan_geometry.segments_[index];
    if (segment.length <= kDistanceTolerance) {
      continue;
    }
    const auto & first = plan_geometry.poses_[index];
    const double ratio = std::clamp(
      ((endpoint.x - first.x) * segment.delta_x +
      (endpoint.y - first.y) * segment.delta_y) /
      (segment.length * segment.length),
      0.0, 1.0);
    const double projected_x = first.x + ratio * segment.delta_x;
    const double projected_y = first.y + ratio * segment.delta_y;
    const double squared_distance =
      (endpoint.x - projected_x) * (endpoint.x - projected_x) +
      (endpoint.y - projected_y) * (endpoint.y - projected_y);
    const double progress =
      segment.start_progress + ratio * segment.length;
    if (squared_distance < nearest_squared_distance - kDistanceTolerance ||
      (std::abs(squared_distance - nearest_squared_distance) <=
      kDistanceTolerance && progress > projection_progress))
    {
      nearest_squared_distance = squared_distance;
      projection_progress = progress;
      projection_segment = index;
    }
  }

  const auto & final_segment =
    plan_geometry.segments_[plan_geometry.last_moving_segment_];
  const double plan_length =
    final_segment.start_progress + final_segment.length;

  // The reference sample uses duplicate-aware vertex tangents. Interpolating
  // those tangents makes the offset curve position and heading continuous at
  // plan bends. Once the finite plan ends, only its last tangent is extended.
  auto reference_segment = projection_segment;
  const auto reference_pose_at =
    [&plan_geometry, plan_length, &reference_segment](
    const double progress, double & heading)
    {
      geometry_msgs::msg::Pose2D reference;
      if (progress >= plan_length - kDistanceTolerance) {
        const std::size_t final_vertex =
          plan_geometry.last_moving_segment_ + 1u;
        heading = plan_geometry.vertex_headings_[final_vertex];
        const double overshoot = std::max(0.0, progress - plan_length);
        reference.x = plan_geometry.poses_[final_vertex].x +
          overshoot * std::cos(heading);
        reference.y = plan_geometry.poses_[final_vertex].y +
          overshoot * std::sin(heading);
        reference.theta = heading;
        return reference;
      }

      while (reference_segment < plan_geometry.segments_.size() &&
        (plan_geometry.segments_[reference_segment].length <=
        kDistanceTolerance ||
        progress >
        plan_geometry.segments_[reference_segment].start_progress +
        plan_geometry.segments_[reference_segment].length +
        kDistanceTolerance))
      {
        ++reference_segment;
      }
      if (reference_segment >= plan_geometry.segments_.size()) {
        throw std::invalid_argument{
                "plan continuation lost monotonic plan progress"};
      }
      const auto & segment = plan_geometry.segments_[reference_segment];
      const double ratio = std::clamp(
        (progress - segment.start_progress) / segment.length, 0.0, 1.0);
      const auto & first = plan_geometry.poses_[reference_segment];
      reference.x = first.x + ratio * segment.delta_x;
      reference.y = first.y + ratio * segment.delta_y;
      heading = interpolate_angle(
        plan_geometry.vertex_headings_[reference_segment],
        plan_geometry.vertex_headings_[reference_segment + 1u], ratio);
      reference.theta = heading;
      return reference;
    };

  double projection_tangent = 0.0;
  const auto projection_reference = reference_pose_at(
    projection_progress, projection_tangent);
  const double projection_sine = std::sin(projection_tangent);
  const double projection_cosine = std::cos(projection_tangent);
  const double initial_lateral_offset =
    -projection_sine * (endpoint.x - projection_reference.x) +
    projection_cosine * (endpoint.y - projection_reference.y);

  // Anchor the offset curve at the exact timed-prefix endpoint. This removes
  // the small seed jump which otherwise appears when the smoothed plan tangent
  // differs from the piecewise-linear projection segment at a bend.
  const double anchor_x = endpoint.x -
    (projection_reference.x - initial_lateral_offset * projection_sine);
  const double anchor_y = endpoint.y -
    (projection_reference.y + initial_lateral_offset * projection_cosine);

  double candidate_heading = projection_tangent;
  const bool heading_is_credible =
    credible_spatial_heading(combined, candidate_heading);
  const double initial_heading_error = heading_is_credible ?
    std::remainder(candidate_heading - projection_tangent, 2.0 * M_PI) : 0.0;

  // Advance plan station monotonically in the same spatial increments used by
  // the critic. The native prefix remains untouched. The heading error is
  // first held through a bounded maneuver, then relaxed over one fixed
  // physical distance; its sin integral creates the lateral separation needed
  // to rank an early detour. This continuation is soft scoring only; the
  // native rollout remains the independent hard gate.
  for (std::size_t step = 1u; step < kMaximumRiskPathSamples; ++step) {
    const double continuation_distance =
      static_cast<double>(step) * sample_resolution;
    if (!std::isfinite(continuation_distance)) {
      throw std::invalid_argument{
              "plan continuation distance overflowed"};
    }
    double reference_heading = projection_tangent;
    const auto reference = reference_pose_at(
      projection_progress + continuation_distance, reference_heading);
    const double heading_error = relaxed_heading_error(
      initial_heading_error, continuation_distance,
      heading_relaxation_distance);
    const double lateral_offset = initial_lateral_offset +
      relaxed_lateral_growth(
      initial_heading_error, continuation_distance,
      heading_relaxation_distance);
    geometry_msgs::msg::Pose2D continued;
    continued.x = anchor_x + reference.x -
      lateral_offset * std::sin(reference_heading);
    continued.y = anchor_y + reference.y +
      lateral_offset * std::cos(reference_heading);
    continued.theta = reference_heading + heading_error;

    const auto previous = combined.poses.back();
    if (!append_if_spatially_distinct(combined, continued)) {
      continue;
    }
    combined_distance += std::hypot(
      continued.x - previous.x, continued.y - previous.y);
    if (!std::isfinite(combined_distance)) {
      throw std::invalid_argument{
              "plan continuation path length overflowed"};
    }
    if (combined_distance + kDistanceTolerance >= risk_distance) {
      return build_fixed_distance_risk_path(
        combined, risk_distance, sample_resolution);
    }
  }

  throw std::invalid_argument{
          "plan continuation exceeded the fixed sample limit"};
}

}  // namespace f_dwa_controller
