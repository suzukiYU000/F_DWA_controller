// Copyright 2026 YTLab
// Licensed under the MIT License.

#ifndef DISPATCH_EPOCH_GATE_HPP_
#define DISPATCH_EPOCH_GATE_HPP_

namespace f_dwa_controller
{
namespace detail
{

enum class DispatchEpochAction
{
  kIgnoreBeforeResetBoundary,
  kApplyResetBoundary,
  kCorrelateCurrentSequence,
};

// A newly configured controller can receive transient-local samples and
// callbacks retained by the executor from the preceding lifecycle epoch.
// Sequenced samples have no valid ledger until the transport's explicit
// unsequenced reset boundary is observed.
constexpr DispatchEpochAction classify_dispatch_epoch(
  const bool has_sequence, const bool reset_boundary_observed) noexcept
{
  if (!reset_boundary_observed && has_sequence) {
    return DispatchEpochAction::kIgnoreBeforeResetBoundary;
  }
  if (!has_sequence) {
    return DispatchEpochAction::kApplyResetBoundary;
  }
  return DispatchEpochAction::kCorrelateCurrentSequence;
}

}  // namespace detail
}  // namespace f_dwa_controller

#endif  // DISPATCH_EPOCH_GATE_HPP_
