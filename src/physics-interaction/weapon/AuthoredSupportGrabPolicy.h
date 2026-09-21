#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::authored_support_grab_policy
{
    [[nodiscard]] constexpr std::uint64_t resolveAuthoredGenerationKey(
        const std::uint64_t collisionGenerationKey,
        const std::uint64_t weaponOwnershipKey) noexcept
    {
        return collisionGenerationKey != 0 ?
            collisionGenerationKey :
            weaponOwnershipKey;
    }

    // Qualification is measured in elapsed ready-state time, never frame
    // count. It gives a new weapon/topology a bounded opportunity to publish
    // its authored support relation before absence becomes dynamic fallback
    // authority. Positive authored data always recovers immediately.
    inline constexpr float kQualificationSeconds = 0.20f;
    inline constexpr float kUsableCandidateLossSeconds = 0.20f;

    [[nodiscard]] inline float advanceContinuousEvidenceSeconds(
        const float currentSeconds,
        const float deltaSeconds,
        const float limitSeconds,
        const bool ready) noexcept
    {
        if (!ready || !std::isfinite(limitSeconds) ||
            limitSeconds <= 0.0f) {
            return 0.0f;
        }
        const float current =
            std::isfinite(currentSeconds) && currentSeconds > 0.0f ?
            (std::min)(currentSeconds, limitSeconds) :
            0.0f;
        if (!std::isfinite(deltaSeconds) || deltaSeconds <= 0.0f) {
            return current;
        }
        return (std::min)(current + deltaSeconds, limitSeconds);
    }

    enum class Capability : std::uint8_t
    {
        Pending,
        Usable,
        Unavailable,
    };

    enum class CapabilityReason : std::uint8_t
    {
        ModeDisabled,
        AwaitingIdentity,
        AwaitingCandidate,
        AwaitingQualification,
        AuthoredPoseUsable,
        UnsupportedWeaponFamily,
        UnclassifiedWeaponFamily,
        SupportTopologyUnavailable,
        CanonicalAxesUnavailable,
        CompleteFingerPoseUnavailable,
        CaptureUnavailableAfterQualification,
        NoAuthoredSupportPose,
    };

    enum class LeftFiringTakeoverReadiness : std::uint8_t
    {
        NotRequired,
        AwaitingFinalGeneration,
        AwaitingCapability,
        AwaitingMirroredCandidate,
        ReadyWithAuthoredCandidate,
        ReadyWithDynamicFallback,
    };

    struct LeftFiringTakeoverReadinessInput
    {
        bool targetFiringHandIsLeft{ false };
        bool authoredOnlyModeEnabled{ false };
        // Authored generation identity (resolveAuthoredGenerationKey): the raw
        // collision generation once built, the ownership key while it is still
        // building. Authored grip data is deliberately collision-independent,
        // so a left takeover must not wait for generated collision; the carry
        // session migrates when the raw generation lands.
        std::uint64_t authoredGenerationKey{ 0 };
        bool capabilityIdentityCurrent{ false };
        Capability capability{ Capability::Pending };
        bool supportPoseAbsent{ false };
        bool mirroredCandidateAvailable{ false };
    };

    [[nodiscard]] constexpr LeftFiringTakeoverReadiness
    resolveLeftFiringTakeoverReadiness(
        const LeftFiringTakeoverReadinessInput& input) noexcept
    {
        if (!input.targetFiringHandIsLeft ||
            !input.authoredOnlyModeEnabled) {
            return LeftFiringTakeoverReadiness::NotRequired;
        }
        if (input.authoredGenerationKey == 0) {
            return LeftFiringTakeoverReadiness::AwaitingFinalGeneration;
        }
        if (!input.capabilityIdentityCurrent) {
            return LeftFiringTakeoverReadiness::AwaitingCapability;
        }
        if (input.capability == Capability::Unavailable && input.supportPoseAbsent) {
            return LeftFiringTakeoverReadiness::ReadyWithDynamicFallback;
        }
        if (input.capability == Capability::Usable &&
            input.mirroredCandidateAvailable) {
            return LeftFiringTakeoverReadiness::ReadyWithAuthoredCandidate;
        }
        return input.capability == Capability::Usable ?
            LeftFiringTakeoverReadiness::AwaitingMirroredCandidate :
            LeftFiringTakeoverReadiness::AwaitingCapability;
    }

    [[nodiscard]] constexpr bool leftFiringTakeoverReady(
        const LeftFiringTakeoverReadiness readiness) noexcept
    {
        return readiness == LeftFiringTakeoverReadiness::NotRequired ||
               readiness ==
                   LeftFiringTakeoverReadiness::ReadyWithAuthoredCandidate ||
               readiness ==
                   LeftFiringTakeoverReadiness::ReadyWithDynamicFallback;
    }

    [[nodiscard]] constexpr const char* leftFiringTakeoverReadinessName(
        const LeftFiringTakeoverReadiness readiness) noexcept
    {
        switch (readiness) {
        case LeftFiringTakeoverReadiness::NotRequired:
            return "not-required";
        case LeftFiringTakeoverReadiness::AwaitingFinalGeneration:
            return "awaiting-final-generation";
        case LeftFiringTakeoverReadiness::AwaitingCapability:
            return "awaiting-capability";
        case LeftFiringTakeoverReadiness::AwaitingMirroredCandidate:
            return "awaiting-mirrored-candidate";
        case LeftFiringTakeoverReadiness::ReadyWithAuthoredCandidate:
            return "ready-authored";
        case LeftFiringTakeoverReadiness::ReadyWithDynamicFallback:
            return "ready-dynamic-fallback";
        }
        return "unknown";
    }

    struct CapabilityObservationInput
    {
        bool modeEnabled{ false };
        bool identityCurrent{ false };
        bool qualificationExpired{ false };
        bool candidatePublished{ false };
        bool candidateResolvedForSupportHand{ false };
        bool weaponFamilyKnown{ false };
        bool weaponFamilySupported{ false };
        bool canonicalAxesValid{ false };
        bool completeFingerPose{ false };
        bool supportPoseAbsent{ false };
    };

    struct CapabilityObservation
    {
        Capability capability{ Capability::Pending };
        CapabilityReason reason{ CapabilityReason::AwaitingIdentity };
    };

    [[nodiscard]] constexpr CapabilityObservation observeCapability(
        const CapabilityObservationInput& input) noexcept
    {
        if (!input.modeEnabled) {
            return {
                .capability = Capability::Pending,
                .reason = CapabilityReason::ModeDisabled,
            };
        }
        if (!input.identityCurrent) {
            return {
                .capability = Capability::Pending,
                .reason = CapabilityReason::AwaitingIdentity,
            };
        }
        if (input.supportPoseAbsent) {
            return {Capability::Unavailable, CapabilityReason::NoAuthoredSupportPose};
        }
        // Captured authored data is authoritative immediately. Generated
        // collision never qualifies or rejects the canonical pose or mirror.
        if (input.candidateResolvedForSupportHand &&
            input.weaponFamilySupported &&
            input.canonicalAxesValid &&
            input.completeFingerPose) {
            return {
                .capability = Capability::Usable,
                .reason = CapabilityReason::AuthoredPoseUsable,
            };
        }

        if (!input.qualificationExpired) {
            return {
                .capability = Capability::Pending,
                .reason = input.candidatePublished ?
                    CapabilityReason::AwaitingQualification :
                    CapabilityReason::AwaitingCandidate,
            };
        }
        if (!input.candidatePublished) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::CaptureUnavailableAfterQualification,
            };
        }
        if (!input.candidateResolvedForSupportHand) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::SupportTopologyUnavailable,
            };
        }
        if (!input.weaponFamilyKnown) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::UnclassifiedWeaponFamily,
            };
        }
        if (!input.weaponFamilySupported) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::UnsupportedWeaponFamily,
            };
        }
        if (!input.canonicalAxesValid) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::CanonicalAxesUnavailable,
            };
        }
        if (!input.completeFingerPose) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::CompleteFingerPoseUnavailable,
            };
        }
        return {
            .capability = Capability::Usable,
            .reason = CapabilityReason::AuthoredPoseUsable,
        };
    }

    enum class Selection : std::uint8_t
    {
        None,
        Failure,
        Reject,
        ProviderDynamic,
        DynamicHandoff,
        Authored,
        DynamicFallback,
        DynamicUnrestricted,
    };

    enum class SelectionReason : std::uint8_t
    {
        None,
        CapabilityPending,
        AuthoredActivationRejected,
        ProviderTargetMatched,
        AmbidextrousHandoffGrip,
        AuthoredCaptureEligible,
        AuthoredCapabilityUnavailable,
        ModeDisabled,
    };

    struct SelectionInput
    {
        bool modeEnabled{ false };
        bool providerPartAuthorityActive{ false };
        bool dynamicHandoffCaptureEligible{ false };
        bool authoredCaptureEligible{ false };
        Capability capability{ Capability::Pending };
        bool supportPoseAbsent{ false };
    };

    struct SelectionDecision
    {
        Selection selection{ Selection::None };
        SelectionReason reason{ SelectionReason::None };
    };

    [[nodiscard]] constexpr SelectionDecision select(
        const SelectionInput& input) noexcept
    {
        // Exact provider part authority is the public override boundary. It
        // remains ahead of every ROCK-local authored/dynamic policy.
        if (input.providerPartAuthorityActive) {
            return {
                .selection = Selection::ProviderDynamic,
                .reason = SelectionReason::ProviderTargetMatched,
            };
        }
        // The firing-grip handoff station is a separate, tightly bounded
        // dynamic acquisition supplied by the ambidextrous state machine. Its
        // caller retains an authored seat when that seat is already at the
        // firing grip, so this branch covers only the missing dynamic station.
        if (input.dynamicHandoffCaptureEligible) {
            return {
                .selection = Selection::DynamicHandoff,
                .reason = SelectionReason::AmbidextrousHandoffGrip,
            };
        }
        if (input.authoredCaptureEligible) {
            return {
                .selection = Selection::Authored,
                .reason = SelectionReason::AuthoredCaptureEligible,
            };
        }
        if (!input.modeEnabled) {
            return {
                .selection = Selection::DynamicUnrestricted,
                .reason = SelectionReason::ModeDisabled,
            };
        }
        if (input.capability == Capability::Unavailable && input.supportPoseAbsent) {
            return {
                .selection = Selection::DynamicFallback,
                .reason = SelectionReason::AuthoredCapabilityUnavailable,
            };
        }
        return {
            .selection = Selection::Reject,
            .reason = input.capability == Capability::Pending ?
                SelectionReason::CapabilityPending :
                SelectionReason::AuthoredActivationRejected,
        };
    }

    struct SurfaceQueryInput
    {
        bool modeEnabled{ false };
        bool providerTargetsActive{ false };
        bool supportPoseAbsenceCurrent{ false };
    };

    [[nodiscard]] constexpr bool requiresSurfaceQueries(const SurfaceQueryInput& input) noexcept
    {
        // Authored seats and firing-grip handoffs supply their own activation
        // zones. Only a possible mesh-based acquisition needs a part search.
        const auto selection = select({
            .modeEnabled = input.modeEnabled,
            .providerPartAuthorityActive = input.providerTargetsActive,
            .capability = input.supportPoseAbsenceCurrent ? Capability::Unavailable : Capability::Pending,
            .supportPoseAbsent = input.supportPoseAbsenceCurrent,
        }).selection;
        return selection == Selection::ProviderDynamic ||
               selection == Selection::DynamicFallback ||
               selection == Selection::DynamicUnrestricted;
    }

    [[nodiscard]] constexpr bool captured(const Selection selection) noexcept
    {
        return selection == Selection::ProviderDynamic ||
               selection == Selection::DynamicHandoff ||
               selection == Selection::Authored ||
               selection == Selection::DynamicFallback ||
               selection == Selection::DynamicUnrestricted;
    }

    [[nodiscard]] constexpr const char* capabilityName(
        const Capability capability) noexcept
    {
        switch (capability) {
        case Capability::Pending:
            return "PENDING";
        case Capability::Usable:
            return "USABLE";
        case Capability::Unavailable:
            return "UNAVAILABLE";
        }
        return "UNKNOWN";
    }

    [[nodiscard]] constexpr const char* capabilityReasonName(
        const CapabilityReason reason) noexcept
    {
        switch (reason) {
        case CapabilityReason::ModeDisabled:
            return "mode-disabled";
        case CapabilityReason::AwaitingIdentity:
            return "awaiting-identity";
        case CapabilityReason::AwaitingCandidate:
            return "awaiting-candidate";
        case CapabilityReason::AwaitingQualification:
            return "awaiting-qualification";
        case CapabilityReason::AuthoredPoseUsable:
            return "authored-pose-usable";
        case CapabilityReason::UnsupportedWeaponFamily:
            return "unsupported-weapon-family";
        case CapabilityReason::UnclassifiedWeaponFamily:
            return "unclassified-weapon-family";
        case CapabilityReason::SupportTopologyUnavailable:
            return "support-topology-unavailable";
        case CapabilityReason::CanonicalAxesUnavailable:
            return "canonical-axes-unavailable";
        case CapabilityReason::CompleteFingerPoseUnavailable:
            return "complete-finger-pose-unavailable";
        case CapabilityReason::CaptureUnavailableAfterQualification:
            return "capture-unavailable-after-qualification";
        case CapabilityReason::NoAuthoredSupportPose:
            return "no-authored-support-pose";
        }
        return "unknown";
    }

    [[nodiscard]] constexpr const char* selectionName(
        const Selection selection) noexcept
    {
        switch (selection) {
        case Selection::None:
            return "NONE";
        case Selection::Failure:
            return "FAILURE";
        case Selection::Reject:
            return "REJECT";
        case Selection::ProviderDynamic:
            return "PROVIDER";
        case Selection::DynamicHandoff:
            return "DYNAMIC_HANDOFF";
        case Selection::Authored:
            return "AUTHORED";
        case Selection::DynamicFallback:
            return "DYNAMIC_FALLBACK";
        case Selection::DynamicUnrestricted:
            return "DYNAMIC_UNRESTRICTED";
        }
        return "UNKNOWN";
    }

    [[nodiscard]] constexpr const char* selectionReasonName(
        const SelectionReason reason) noexcept
    {
        switch (reason) {
        case SelectionReason::None:
            return "none";
        case SelectionReason::CapabilityPending:
            return "capability-pending";
        case SelectionReason::AuthoredActivationRejected:
            return "authored-activation-rejected";
        case SelectionReason::ProviderTargetMatched:
            return "provider-target-matched";
        case SelectionReason::AmbidextrousHandoffGrip:
            return "ambidextrous-handoff-firing-grip";
        case SelectionReason::AuthoredCaptureEligible:
            return "authored-capture-eligible";
        case SelectionReason::AuthoredCapabilityUnavailable:
            return "authored-capability-unavailable";
        case SelectionReason::ModeDisabled:
            return "mode-disabled";
        }
        return "unknown";
    }
}
