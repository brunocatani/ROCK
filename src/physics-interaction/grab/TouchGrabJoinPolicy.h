#pragma once

#include "api/ProviderRuntimeTypes.h"

#include <cstdint>

namespace rock::touch_grab_join_policy
{
    [[nodiscard]] inline constexpr std::uint32_t flag(
        const ::rock::api::touch::TouchGrabTargetFlag value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    [[nodiscard]] inline constexpr bool hasFlag(
        const ::rock::api::touch::TouchGrabTarget& target,
        const ::rock::api::touch::TouchGrabTargetFlag value) noexcept
    {
        return (target.flags & flag(value)) != 0;
    }

    /*
     * Separate per-hand wildcard descriptors preserve independent-body grabs.
     * They may share an already active FixedAnchor body only when both opt into
     * two-hand ownership and their non-hand runtime contracts are identical.
     */
    [[nodiscard]] inline constexpr bool canJoinSameBody(
        const std::uint64_t activeOwnerToken,
        const std::uint64_t activeScopeToken,
        const ::rock::api::touch::TouchGrabTarget& activeTarget,
        const std::uint64_t candidateOwnerToken,
        const std::uint64_t candidateScopeToken,
        const ::rock::api::touch::TouchGrabTarget& candidateTarget)
        noexcept
    {
        using Flag = ::rock::api::touch::TouchGrabTargetFlag;
        using Kind = ::rock::api::touch::TouchGrabKind;

        constexpr std::uint32_t handPermissionMask =
            flag(Flag::AllowRightHand) |
            flag(Flag::AllowLeftHand);
        const std::uint32_t activeContractFlags =
            activeTarget.flags & ~handPermissionMask;
        const std::uint32_t candidateContractFlags =
            candidateTarget.flags & ~handPermissionMask;

        return activeOwnerToken != 0 &&
               activeOwnerToken == candidateOwnerToken &&
               activeScopeToken != 0 &&
               activeScopeToken == candidateScopeToken &&
               activeTarget.kind == Kind::FixedAnchor &&
               candidateTarget.kind == Kind::FixedAnchor &&
               hasFlag(activeTarget, Flag::MatchAnyBody) &&
               hasFlag(candidateTarget, Flag::MatchAnyBody) &&
               hasFlag(activeTarget, Flag::AllowTwoHands) &&
               hasFlag(candidateTarget, Flag::AllowTwoHands) &&
               activeTarget.targetGeneration ==
                   candidateTarget.targetGeneration &&
               activeContractFlags == candidateContractFlags &&
               activeTarget.bodyId == candidateTarget.bodyId &&
               activeTarget.referenceFormId ==
                   candidateTarget.referenceFormId &&
               activeTarget.referenceNativeHandle ==
                   candidateTarget.referenceNativeHandle &&
               activeTarget.allowedLayerMask ==
                   candidateTarget.allowedLayerMask &&
               activeTarget.worldGeneration ==
                   candidateTarget.worldGeneration &&
               activeTarget.skeletonGeneration ==
                   candidateTarget.skeletonGeneration &&
               activeTarget.providerGeneration ==
                   candidateTarget.providerGeneration;
    }
}
