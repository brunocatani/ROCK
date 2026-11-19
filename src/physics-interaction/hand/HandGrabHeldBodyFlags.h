// Body flags and activation of a held object's bodies: the flags a hold sets,
// the leases that own them, and their release.
#pragma once

#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock::hand_grab_internal
{
    inline constexpr const char* kHeldObjectDriveName = "proxyConstraint";

    constexpr std::uint32_t heldCollisionParticipationFlags(grab_target::Kind targetKind)
    {
        // Key-2 manifold events (0x40, also used by hands/bipod) report a
        // loose prop's sustained surface contact even without an impact.
        // Other grab kinds keep their existing notification contract.
        return 0x80u | (targetKind == grab_target::Kind::LooseObject ? 0x40u : 0u);
    }

    inline constexpr std::uint32_t kHeldCollisionParticipationFlagMode = 0u;

    inline constexpr std::uint32_t kHeldAuthorityBodyFlags = 0x08000000u;

    inline constexpr std::uint32_t kHeldAuthorityBodyFlagMode = 1u;

    inline std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand)
    {
        return reinterpret_cast<std::uintptr_t>(hand) ^ 0x524F434B48454C44ull;
    }

    struct HeldBodyActivationSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t activatedCount = 0;
        std::uint32_t failedActivationCount = 0;
    };

    struct HeldBodyFlagLeaseSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t collisionLeaseCount = 0;
        std::uint32_t authorityLeaseCount = 0;
        std::uint32_t failedLeaseCount = 0;
    };

    inline HeldBodyActivationSummary activateHeldObjectBodySet(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds)
    {
        HeldBodyActivationSummary summary{};
        if (!world) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (physics_recursive_wrappers::activateBody(world, bodyId)) {
                ++summary.activatedCount;
            } else {
                ++summary.failedActivationCount;
            }
        }
        return summary;
    }

    inline HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, grab_target::Kind targetKind, std::uintptr_t ownerToken)
    {
        /*
             * Proxy-constraint grab replaced the native held-object action, but the
             * old path owned two different flag contracts: collision flags were
             * leased across the accepted held body set by ROCK, while the native action leased
             * 0x08000000 only on its selected primary body. Keeping that split is
             * important for multipart weapons because secondary collision bodies
             * should participate in the hold without all becoming grab-authority
             * bodies.
             */
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::acquireBodyFlagLease(world, bodyId, heldCollisionParticipationFlags(targetKind),
                    kHeldCollisionParticipationFlagMode, ownerToken)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }

        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::acquireBodyFlagLease(world, primaryBodyId, kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode, ownerToken)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }

    inline HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, grab_target::Kind targetKind, std::uintptr_t ownerToken,
        bool restoreOnFinalLease)
    {
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::releaseBodyFlagLease(world, bodyId, heldCollisionParticipationFlags(targetKind),
                    kHeldCollisionParticipationFlagMode, ownerToken, restoreOnFinalLease)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }

        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::releaseBodyFlagLease(world, primaryBodyId, kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode, ownerToken, restoreOnFinalLease)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }
}
