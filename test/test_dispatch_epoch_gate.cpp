// Copyright 2026 YTLab
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <cstdint>

#include "dispatch_epoch_gate.hpp"

namespace f_dwa_controller
{
namespace detail
{
namespace
{

TEST(DispatchEpochGate, IgnoresReplayedSequencesUntilExplicitResetBoundary)
{
  bool reset_boundary_observed = false;

  for (uint64_t old_sequence = 0; old_sequence < 15; ++old_sequence) {
    (void)old_sequence;
    EXPECT_EQ(
      classify_dispatch_epoch(true, reset_boundary_observed),
      DispatchEpochAction::kIgnoreBeforeResetBoundary);
    // Ignoring a stale sample must not implicitly arm the new epoch.
    EXPECT_FALSE(reset_boundary_observed);
  }

  EXPECT_EQ(
    classify_dispatch_epoch(false, reset_boundary_observed),
    DispatchEpochAction::kApplyResetBoundary);
  reset_boundary_observed = true;

  EXPECT_EQ(
    classify_dispatch_epoch(true, reset_boundary_observed),
    DispatchEpochAction::kCorrelateCurrentSequence);
}

TEST(DispatchEpochGate, RepeatedResetRemainsAnExplicitBoundary)
{
  EXPECT_EQ(
    classify_dispatch_epoch(false, true),
    DispatchEpochAction::kApplyResetBoundary);
}

}  // namespace
}  // namespace detail
}  // namespace f_dwa_controller
