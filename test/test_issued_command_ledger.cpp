// Copyright 2026 YTLab
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>

#include "f_dwa_controller/issued_command_ledger.hpp"

namespace f_dwa_controller
{
namespace
{

IssuedCommandLedgerEntry entry(
  const int64_t ros_nanoseconds, const uint64_t steady_nanoseconds,
  const double linear, const double angular)
{
  IssuedCommandLedgerEntry value;
  value.issued_at = rclcpp::Time(ros_nanoseconds, RCL_ROS_TIME);
  value.issued_steady_time_ns = steady_nanoseconds;
  value.command.x = linear;
  value.command.theta = angular;
  return value;
}

TEST(IssuedCommandLedger, DelayedDispatchCannotMatchIdenticalFutureCommand)
{
  std::deque<IssuedCommandLedgerEntry> commands{
    entry(200, 90u, 0.06, 0.0),
    // Issued after the old command entered the FIFO at t=100, but before that
    // old command was finally dispatched. Deliberately give this future
    // command an earlier ROS timestamp: /clock cache skew must not admit it.
    entry(50, 150u, 0.60, 0.19)};
  nav_2d_msgs::msg::Twist2D dispatched;
  dispatched.x = 0.60;
  dispatched.theta = 0.19;

  EXPECT_EQ(eligible_command_prefix_size(
      commands, 100u), 1u);
  EXPECT_FALSE(find_eligible_command_index(
      commands, 100u, dispatched).has_value());
}

TEST(IssuedCommandLedger, FindsMatchingCommandInsideEligiblePrefix)
{
  std::deque<IssuedCommandLedgerEntry> commands{
    entry(80, 80u, 0.06, 0.0),
    entry(90, 90u, 0.60, 0.19),
    entry(110, 110u, 0.60, 0.19)};
  nav_2d_msgs::msg::Twist2D dispatched;
  dispatched.x = 0.60;
  dispatched.theta = 0.19;

  const auto match = find_eligible_command_index(
    commands, 100u, dispatched);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(*match, 1u);
}

TEST(IssuedCommandLedger, ExternalZeroCannotConsumeFuturePluginZero)
{
  std::deque<IssuedCommandLedgerEntry> commands{
    entry(50, 150u, 0.0, 0.0)};
  nav_2d_msgs::msg::Twist2D dispatched_zero;

  EXPECT_FALSE(find_eligible_command_index(
      commands, 100u, dispatched_zero).has_value());
}

TEST(IssuedCommandLedger, RosClockSkewDoesNotHideCausallyEligibleCommand)
{
  std::deque<IssuedCommandLedgerEntry> commands{
    // A different process may have observed a newer /clock sample at issue
    // time than the transport later has cached at FIFO reception.
    entry(150, 80u, 0.06, 0.0)};
  nav_2d_msgs::msg::Twist2D dispatched;
  dispatched.x = 0.06;

  EXPECT_EQ(eligible_command_prefix_size(
      commands, 100u), 1u);
  const auto match = find_eligible_command_index(commands, 100u, dispatched);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(*match, 0u);
}

TEST(IssuedCommandLedger, MissingMonotonicProvenanceIsNeverEligible)
{
  std::deque<IssuedCommandLedgerEntry> commands{
    entry(80, 0u, 0.06, 0.0)};
  nav_2d_msgs::msg::Twist2D dispatched;
  dispatched.x = 0.06;

  EXPECT_EQ(eligible_command_prefix_size(commands, 100u), 0u);
  EXPECT_FALSE(find_eligible_command_index(
      commands, 100u, dispatched).has_value());
  commands.front().issued_steady_time_ns = 80u;
  EXPECT_EQ(eligible_command_prefix_size(commands, 0u), 0u);
}

}  // namespace
}  // namespace f_dwa_controller
