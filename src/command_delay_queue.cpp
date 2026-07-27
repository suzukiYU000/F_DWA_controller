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

#include "f_dwa_controller/command_delay_queue.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "rclcpp/duration.hpp"

namespace f_dwa_controller
{

namespace
{

constexpr std::size_t kMaxSamplingAttempts = 1000;

void validate_parameters(const CommandDelayParameters & parameters)
{
  if (!std::isfinite(parameters.min_delay_ms) ||
    !std::isfinite(parameters.max_delay_ms) ||
    !std::isfinite(parameters.mean_delay_ms) ||
    !std::isfinite(parameters.delay_stddev_ms))
  {
    throw std::invalid_argument("Delay parameters must be finite");
  }
  if (parameters.min_delay_ms < 0.0 ||
    parameters.max_delay_ms < parameters.min_delay_ms)
  {
    throw std::invalid_argument("Delay bounds are invalid");
  }
  if (parameters.mean_delay_ms < parameters.min_delay_ms ||
    parameters.mean_delay_ms > parameters.max_delay_ms)
  {
    throw std::invalid_argument("Mean delay must be inside the delay bounds");
  }
  if (parameters.delay_stddev_ms < 0.0) {
    throw std::invalid_argument("Delay standard deviation must be non-negative");
  }
  if (!std::isfinite(parameters.zero_threshold) || parameters.zero_threshold < 0.0) {
    throw std::invalid_argument("Zero threshold must be finite and non-negative");
  }
  if (parameters.max_queue_depth == 0) {
    throw std::invalid_argument("Maximum queue depth must be positive");
  }
}

double zero_if_small(const double value, const double threshold)
{
  return std::abs(value) < threshold ? 0.0 : value;
}

}  // namespace

CommandDelayQueue::CommandDelayQueue(const CommandDelayParameters & parameters)
: parameters_(parameters),
  random_engine_(parameters.random_seed),
  delay_distribution_(
    parameters.mean_delay_ms,
    parameters.delay_stddev_ms > 0.0 ? parameters.delay_stddev_ms : 1.0)
{
  validate_parameters(parameters_);
}

bool CommandDelayQueue::enqueue(
  const geometry_msgs::msg::Twist & command,
  const rclcpp::Time & received_at)
{
  if (queue_.size() >= parameters_.max_queue_depth) {
    return false;
  }

  const double delay_ms = sample_delay_ms();
  const auto delay = rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(std::llround(delay_ms * 1.0e6)));
  queue_.push_back(
    DelayedCommand{
      normalized_command(command, parameters_.zero_threshold),
      received_at,
      received_at + delay,
      next_sequence_,
      delay_ms});
  ++next_sequence_;
  return true;
}

std::optional<DelayedCommand> CommandDelayQueue::pop_due(const rclcpp::Time & now)
{
  if (queue_.empty() || queue_.front().eligible_at > now) {
    return std::nullopt;
  }

  DelayedCommand command = queue_.front();
  queue_.pop_front();
  return command;
}

std::size_t CommandDelayQueue::size() const
{
  return queue_.size();
}

bool CommandDelayQueue::empty() const
{
  return queue_.empty();
}

const DelayedCommand * CommandDelayQueue::front() const
{
  return queue_.empty() ? nullptr : &queue_.front();
}

uint64_t CommandDelayQueue::next_sequence() const
{
  return next_sequence_;
}

void CommandDelayQueue::clear()
{
  queue_.clear();
}

geometry_msgs::msg::Twist CommandDelayQueue::normalized_command(
  const geometry_msgs::msg::Twist & command,
  const double zero_threshold)
{
  geometry_msgs::msg::Twist normalized = command;
  normalized.linear.x = zero_if_small(command.linear.x, zero_threshold);
  normalized.linear.y = zero_if_small(command.linear.y, zero_threshold);
  normalized.linear.z = zero_if_small(command.linear.z, zero_threshold);
  normalized.angular.x = zero_if_small(command.angular.x, zero_threshold);
  normalized.angular.y = zero_if_small(command.angular.y, zero_threshold);
  normalized.angular.z = zero_if_small(command.angular.z, zero_threshold);
  return normalized;
}

bool CommandDelayQueue::is_zero(
  const geometry_msgs::msg::Twist & command,
  const double zero_threshold)
{
  const std::array<double, 6> components{
    command.linear.x,
    command.linear.y,
    command.linear.z,
    command.angular.x,
    command.angular.y,
    command.angular.z};
  return std::all_of(
    components.cbegin(), components.cend(),
    [zero_threshold](const double value) {
      return std::abs(value) < zero_threshold;
    });
}

double CommandDelayQueue::sample_delay_ms()
{
  if (parameters_.delay_stddev_ms == 0.0) {
    return parameters_.mean_delay_ms;
  }

  for (std::size_t attempt = 0; attempt < kMaxSamplingAttempts; ++attempt) {
    const double sample = delay_distribution_(random_engine_);
    if (sample >= parameters_.min_delay_ms && sample <= parameters_.max_delay_ms) {
      return sample;
    }
  }

  // The configured default bounds are approximately ±3 sigma. Reaching this
  // fallback means a custom distribution is numerically too narrow to sample
  // efficiently, but clamping still preserves the configured hard bounds.
  return std::clamp(
    parameters_.mean_delay_ms,
    parameters_.min_delay_ms,
    parameters_.max_delay_ms);
}

}  // namespace f_dwa_controller
