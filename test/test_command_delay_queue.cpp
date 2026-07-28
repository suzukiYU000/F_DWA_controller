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

#include <cmath>
#include <cstddef>

#include "f_dwa_controller/command_delay_queue.hpp"
#include "gtest/gtest.h"
#include "rclcpp/time.hpp"

namespace f_dwa_controller
{

TEST(CommandDelayQueue, AppliesFixedDelayInFifoOrder)
{
  CommandDelayParameters parameters;
  parameters.mean_delay_ms = 70.0;
  parameters.delay_stddev_ms = 0.0;
  parameters.max_queue_depth = 4;
  CommandDelayQueue queue(parameters);

  geometry_msgs::msg::Twist first;
  first.linear.x = 0.2;
  geometry_msgs::msg::Twist second;
  second.linear.x = 0.4;

  const rclcpp::Time start(0, 0, RCL_ROS_TIME);
  ASSERT_TRUE(queue.enqueue(first, start));
  ASSERT_TRUE(queue.enqueue(second, start + rclcpp::Duration::from_seconds(0.03)));

  EXPECT_FALSE(queue.pop_due(start + rclcpp::Duration::from_seconds(0.069)).has_value());
  const auto first_due = queue.pop_due(start + rclcpp::Duration::from_seconds(0.070));
  ASSERT_TRUE(first_due.has_value());
  EXPECT_EQ(first_due->sequence, 0u);
  EXPECT_DOUBLE_EQ(first_due->command.linear.x, 0.2);

  EXPECT_FALSE(queue.pop_due(start + rclcpp::Duration::from_seconds(0.099)).has_value());
  const auto second_due = queue.pop_due(start + rclcpp::Duration::from_seconds(0.100));
  ASSERT_TRUE(second_due.has_value());
  EXPECT_EQ(second_due->sequence, 1u);
  EXPECT_DOUBLE_EQ(second_due->command.linear.x, 0.4);
}

TEST(CommandDelayQueue, RejectsOverflowWithoutDroppingQueuedCommands)
{
  CommandDelayParameters parameters;
  parameters.delay_stddev_ms = 0.0;
  parameters.max_queue_depth = 2;
  CommandDelayQueue queue(parameters);
  geometry_msgs::msg::Twist command;
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);

  ASSERT_TRUE(queue.enqueue(command, now));
  ASSERT_TRUE(queue.enqueue(command, now));
  EXPECT_FALSE(queue.enqueue(command, now));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.next_sequence(), 2u);
}

TEST(CommandDelayQueue, TruncatedNormalStaysInsideConfiguredBounds)
{
  CommandDelayParameters parameters;
  parameters.max_queue_depth = 1;
  parameters.random_seed = 42;
  CommandDelayQueue queue(parameters);
  geometry_msgs::msg::Twist command;
  rclcpp::Time received_at(0, 0, RCL_ROS_TIME);

  double delay_sum = 0.0;
  constexpr std::size_t kSamples = 10000;
  for (std::size_t index = 0; index < kSamples; ++index) {
    ASSERT_TRUE(queue.enqueue(command, received_at));
    const DelayedCommand * queued = queue.front();
    ASSERT_NE(queued, nullptr);
    EXPECT_GE(queued->sampled_delay_ms, parameters.min_delay_ms);
    EXPECT_LE(queued->sampled_delay_ms, parameters.max_delay_ms);
    delay_sum += queued->sampled_delay_ms;
    queue.clear();
  }

  EXPECT_NEAR(delay_sum / static_cast<double>(kSamples), parameters.mean_delay_ms, 0.1);
}

TEST(CommandDelayQueue, NormalizesOnlyComponentsInsideStopThreshold)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.009;
  command.linear.y = -0.01;
  command.angular.z = 0.011;

  const auto normalized = CommandDelayQueue::normalized_command(command, 0.01);
  EXPECT_DOUBLE_EQ(normalized.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(normalized.linear.y, -0.01);
  EXPECT_DOUBLE_EQ(normalized.angular.z, 0.011);
  EXPECT_FALSE(CommandDelayQueue::is_zero(command, 0.01));

  command.linear.y = -0.009;
  command.angular.z = 0.0;
  EXPECT_TRUE(CommandDelayQueue::is_zero(command, 0.01));
}

TEST(CommandDelayQueue, TrialResetClearsStateAndRestartsSeed)
{
  CommandDelayParameters parameters;
  parameters.max_queue_depth = 2;
  parameters.random_seed = 42;
  CommandDelayQueue queue(parameters);
  geometry_msgs::msg::Twist command;
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);

  ASSERT_TRUE(queue.enqueue(command, now));
  const DelayedCommand * first = queue.front();
  ASSERT_NE(first, nullptr);
  const double first_delay_ms = first->sampled_delay_ms;
  EXPECT_EQ(first->sequence, 0u);

  queue.reset(parameters.random_seed);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.next_sequence(), 0u);

  ASSERT_TRUE(queue.enqueue(command, now));
  const DelayedCommand * repeated = queue.front();
  ASSERT_NE(repeated, nullptr);
  EXPECT_EQ(repeated->sequence, 0u);
  EXPECT_DOUBLE_EQ(repeated->sampled_delay_ms, first_delay_ms);
}

TEST(CommandDelayQueue, DefaultThresholdPreservesFirStartupIncrement)
{
  CommandDelayParameters parameters;
  parameters.min_delay_ms = 0.0;
  parameters.max_delay_ms = 0.0;
  parameters.mean_delay_ms = 0.0;
  parameters.delay_stddev_ms = 0.0;
  CommandDelayQueue queue(parameters);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.001;
  ASSERT_TRUE(queue.enqueue(command, rclcpp::Time(0, 0, RCL_ROS_TIME)));
  const auto dispatched =
    queue.pop_due(rclcpp::Time(0, 0, RCL_ROS_TIME));
  ASSERT_TRUE(dispatched.has_value());
  EXPECT_DOUBLE_EQ(dispatched->command.linear.x, command.linear.x);
}

}  // namespace f_dwa_controller
