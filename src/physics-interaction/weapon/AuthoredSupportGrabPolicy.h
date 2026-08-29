#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::authored_support_grab_policy
{
    // Qualification is measured in elapsed ready-state time, never frame
    // count. It gives a new weapon/topology a bounded opportunity to publish
    // its authored support relation before absence becomes dynamic fallback
    // authority. An unavailable result is rechecked slowly so a later valid
    // animation sample can recover without a re-equip.
    inline constexpr float kQualificationSeconds = 0.20f;
    inline constexpr float kUsableEvidenceLossSeconds = 0.20f;
    inline constexpr float kUnavailableRecheckSeconds = 1.0f;

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
        AwaitingGeometry,
        AwaitingCandidate,
        AwaitingQualification,
        AuthoredPoseUsable,
        UnsupportedWeaponFamily,
        UnclassifiedWeaponFamily,
        SupportTopologyUnavailable,
        CanonicalAxesUnavailable,
        CompleteFingerPoseUnavailable,
        PoseEvidenceUnavailable,
        PoseOffCurrentWeaponGeometry,
        CaptureUnavailableAfterQualification,
    };

    struct CapabilityObservationInput
    {
        bool modeEnabled{ false };
        bool identityCurrent{ false };
        bool geometryReady{ false };
        bool qualificationExpired{ false };
        bool candidatePublished{ false };
        bool candidateResolvedForSupportHand{ false };
        bool weaponFamilyKnown{ false };
        bool weaponFamilySupported{ false };
        bool canonicalAxesValid{ false };
        bool completeFingerPose{ false };
        bool poseEvidenceEvaluated{ false };
        bool poseEvidencePass{ false };
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
        if (!input.geometryReady) {
            return {
                .capability = Capability::Pending,
                .reason = CapabilityReason::AwaitingGeometry,
            };
        }

        // Positive evidence is authoritative immediately. The timer exists to
        // qualify absence or broken authored data, not to delay a valid seat.
        if (input.candidateResolvedForSupportHand &&
            input.weaponFamilySupported &&
            input.canonicalAxesValid &&
            input.completeFingerPose &&
            input.poseEvidenceEvaluated &&
            input.poseEvidencePass) {
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
        if (!input.poseEvidenceEvaluated) {
            return {
                .capability = Capability::Unavailable,
                .reason = CapabilityReason::PoseEvidenceUnavailable,
            };
        }
        return {
            .capability = Capability::Unavailable,
            .reason = CapabilityReason::PoseOffCurrentWeaponGeometry,
        };
    }

    enum class Selection : std::uint8_t
    {
        None,
        Failure,
        Reject,
        ProviderDynamic,
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
        AuthoredCaptureEligible,
        AuthoredCapabilityUnavailable,
        ModeDisabled,
    };

    struct SelectionInput
    {
        bool modeEnabled{ false };
        bool providerPartAuthorityActive{ false };
        bool authoredCaptureEligible{ false };
        Capability capability{ Capability::Pending };
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
        if (input.capability == Capability::Unavailable) {
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

    [[nodiscard]] constexpr bool captured(const Selection selection) noexcept
    {
        return selection == Selection::ProviderDynamic ||
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
        case CapabilityReason::AwaitingGeometry:
            return "awaiting-geometry";
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
        case CapabilityReason::PoseEvidenceUnavailable:
            return "pose-evidence-unavailable";
        case CapabilityReason::PoseOffCurrentWeaponGeometry:
            return "pose-off-current-weapon-geometry";
        case CapabilityReason::CaptureUnavailableAfterQualification:
            return "capture-unavailable-after-qualification";
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
