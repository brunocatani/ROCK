#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "physics-interaction/visual/HandWorldAuthorityRegistryPolicy.h"
#include "physics-interaction/weapon/collision/WeaponIntentStabilityPolicy.h"

#include "RE/NetImmerse/NiTransform.h"

namespace rock::presentation_trace_policy
{
    /*
     * One equipped-weapon presentation frame, recorded as plain data.
     *
     * The weapon and its attached hands are one visual object, but they are
     * written by different owners at different points of the frame: the
     * dynamic collision correction at ROCK phase 6, hFRIK's deferred hand
     * solve one scheduling interval later, and the gunstock/provider writers
     * after that. A defect in that group shows up as "the weapon moved and
     * the hands did not", which no single call site can observe.
     *
     * This record is the per-frame observation point, and the predicates
     * below are the machine-checkable form of the invariants the presentation
     * group must hold. They are pure so the definition of a violation lives
     * in one testable place instead of inside engine call sites.
     */

    using Role = hand_world_authority_registry_policy::Role;

    enum class InvariantCounter : std::size_t
    {
        // The weapon accepted a collision correction while a required hand
        // claim did not publish. This is the partial-commit defect itself.
        WeaponWriteWithFailedHands = 0,
        // A two-hand group published one hand and failed the other.
        PartialGroupStage,
        // The post-hFRIK independent-weapon restore refused to run.
        RestoreGuardFailure,
        // Something moved the weapon after its colliders and muzzle sampled
        // it. Batch 7 relocates those consumers; this must reach zero.
        LateWeaponWriterAfterColliderPublication,
        // A collision correction reached a hand whose publication channel was
        // not ready, so the claim could never be honoured.
        CorrectionWhilePublicationUnready,
        // A firing recoil delta was composed into the presented weapon.
        RecoilDeltaApplied,
        Count,
    };

    inline constexpr std::size_t kInvariantCounterCount =
        static_cast<std::size_t>(InvariantCounter::Count);

    enum class RestoreGuardFailure : std::uint8_t
    {
        None = 0,
        WeaponNodeMismatch,
        GenerationMismatch,
        SchedulerMismatch,
        ParentHandMismatch,
        NoHandAuthority,
        NoWinner,
        WinnerSequenceChanged,
        WinnerRoleNotWeaponFollowing,
        RigidMoveFailed,
        Count,
    };

    /*
     * A writer that runs after the generated colliders and the muzzle have
     * sampled the weapon leaves them describing a pose the player never sees.
     * These thresholds are well under one rendered pixel of weapon motion, so
     * only a real late write trips the counter.
     */
    inline constexpr float kLateWriterTranslationToleranceGameUnits = 0.02f;
    inline constexpr float kLateWriterRotationToleranceDegrees = 0.05f;

    struct HandRecord
    {
        std::uint64_t winnerSequence = 0;
        int winnerPriority = 0;
        Role winnerRole = Role::Unknown;
        bool winnerValid = false;
        bool publicationReady = false;
        bool collisionRequested = false;
        bool collisionTargetValid = false;
        bool collisionApplied = false;
        bool collisionAuthorityLive = false;
    };

    struct FrameRecord
    {
        std::uint64_t frameIndex = 0;
        std::uint64_t schedulerSequence = 0;
        std::uint64_t weaponGenerationKey = 0;

        bool dwcProxyActive = false;
        bool dwcContactActive = false;
        bool dwcPublishRequested = false;
        bool dwcWeaponPublished = false;
        bool dwcHandGroupPublished = false;
        float dwcContactRetentionSeconds = 0.0f;
        float dwcTranslationCorrectionGameUnits = 0.0f;
        float dwcRotationCorrectionDegrees = 0.0f;
        std::uint32_t dwcOtherBodyId = 0x7FFF'FFFFu;
        std::uint32_t dwcOtherLayer = 0;

        weapon_intent_stability_policy::Sample intentStability{};

        std::uint64_t recoilAcceptedSequence = 0;
        std::uint64_t recoilConsumedSequence = 0;
        bool recoilApplied = false;

        RestoreGuardFailure restoreGuardFailure = RestoreGuardFailure::None;
        bool restoreGuardEvaluated = false;

        bool weaponSampledAtColliderPublication = false;
        RE::NiTransform weaponWorldAtColliderPublication{};
        float lateWriterTranslationGameUnits = -1.0f;
        float lateWriterRotationDegrees = -1.0f;

        std::array<HandRecord, 2> hands{};
        bool valid = false;
    };

    [[nodiscard]] inline constexpr std::size_t handIndex(
        const bool isLeft) noexcept
    {
        return isLeft ? 0u : 1u;
    }

    // The weapon took the correction while the hands it belongs to did not.
    [[nodiscard]] inline constexpr bool violatesGroupCommit(
        const FrameRecord& record) noexcept
    {
        return record.dwcPublishRequested &&
            record.dwcWeaponPublished &&
            !record.dwcHandGroupPublished;
    }

    // One hand of a requested pair took the claim and the other did not.
    [[nodiscard]] inline constexpr bool violatesGroupAtomicity(
        const FrameRecord& record) noexcept
    {
        if (!record.dwcPublishRequested) {
            return false;
        }
        bool anyApplied = false;
        bool anyRequestedNotApplied = false;
        for (const auto& hand : record.hands) {
            if (!hand.collisionRequested) {
                continue;
            }
            anyApplied = anyApplied || hand.collisionApplied;
            anyRequestedNotApplied =
                anyRequestedNotApplied || !hand.collisionApplied;
        }
        return anyApplied && anyRequestedNotApplied;
    }

    // A claim was requested on a hand whose publication channel was closed.
    [[nodiscard]] inline constexpr bool violatesPublicationReadiness(
        const FrameRecord& record) noexcept
    {
        if (!record.dwcPublishRequested) {
            return false;
        }
        for (const auto& hand : record.hands) {
            if (hand.collisionRequested && !hand.publicationReady) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool violatesDownstreamOrdering(
        const FrameRecord& record) noexcept
    {
        if (!record.weaponSampledAtColliderPublication) {
            return false;
        }
        return (std::isfinite(record.lateWriterTranslationGameUnits) &&
                   record.lateWriterTranslationGameUnits >
                       kLateWriterTranslationToleranceGameUnits) ||
            (std::isfinite(record.lateWriterRotationDegrees) &&
                record.lateWriterRotationDegrees >
                    kLateWriterRotationToleranceDegrees);
    }

    [[nodiscard]] inline constexpr const char* counterName(
        const InvariantCounter counter) noexcept
    {
        switch (counter) {
        case InvariantCounter::WeaponWriteWithFailedHands:
            return "weapon-write-with-failed-hands";
        case InvariantCounter::PartialGroupStage:
            return "partial-group-stage";
        case InvariantCounter::RestoreGuardFailure:
            return "restore-guard-failure";
        case InvariantCounter::LateWeaponWriterAfterColliderPublication:
            return "late-weapon-writer-after-collider-publication";
        case InvariantCounter::CorrectionWhilePublicationUnready:
            return "correction-while-publication-unready";
        case InvariantCounter::RecoilDeltaApplied:
            return "recoil-delta-applied";
        default:
            return "unknown";
        }
    }

    [[nodiscard]] inline constexpr const char* restoreGuardFailureName(
        const RestoreGuardFailure reason) noexcept
    {
        switch (reason) {
        case RestoreGuardFailure::None:
            return "none";
        case RestoreGuardFailure::WeaponNodeMismatch:
            return "weapon-node-mismatch";
        case RestoreGuardFailure::GenerationMismatch:
            return "generation-mismatch";
        case RestoreGuardFailure::SchedulerMismatch:
            return "scheduler-mismatch";
        case RestoreGuardFailure::ParentHandMismatch:
            return "parent-hand-mismatch";
        case RestoreGuardFailure::NoHandAuthority:
            return "no-hand-authority";
        case RestoreGuardFailure::NoWinner:
            return "no-winner";
        case RestoreGuardFailure::WinnerSequenceChanged:
            return "winner-sequence-changed";
        case RestoreGuardFailure::WinnerRoleNotWeaponFollowing:
            return "winner-role-not-weapon-following";
        case RestoreGuardFailure::RigidMoveFailed:
            return "rigid-move-failed";
        default:
            return "unknown";
        }
    }

    [[nodiscard]] inline constexpr const char* roleName(
        const Role role) noexcept
    {
        switch (role) {
        case Role::GrabHeld:
            return "grab-held";
        case Role::GrabReturn:
            return "grab-return";
        case Role::EquipHandoff:
            return "equip-handoff";
        case Role::DynamicContact:
            return "dynamic-contact";
        case Role::PrimaryGrip:
            return "primary-grip";
        case Role::SupportGrip:
            return "support-grip";
        case Role::PrimaryDetach:
            return "primary-detach";
        case Role::Gunstock:
            return "gunstock";
        case Role::WeaponCollision:
            return "weapon-collision";
        case Role::WeaponReturn:
            return "weapon-return";
        case Role::Provider:
            return "provider";
        case Role::ProviderWeaponCoupled:
            return "provider-weapon-coupled";
        default:
            return "unknown";
        }
    }
}
