#include "physics-interaction/weapon/AuthoredSupportGrabPolicy.h"

#include <cassert>
#include <limits>

using namespace rock::authored_support_grab_policy;

namespace
{
    constexpr CapabilityObservationInput usableInput{
        .modeEnabled = true,
        .identityCurrent = true,
        .qualificationExpired = true,
        .candidatePublished = true,
        .candidateResolvedForSupportHand = true,
        .weaponFamilyKnown = true,
        .weaponFamilySupported = true,
        .canonicalAxesValid = true,
        .completeFingerPose = true,
    };
}

int main()
{
    static_assert(resolveAuthoredGenerationKey(11, 22) == 11);
    static_assert(resolveAuthoredGenerationKey(0, 22) == 22);
    static_assert(resolveAuthoredGenerationKey(0, 0) == 0);
    static_assert(observeCapability(usableInput).capability ==
                  Capability::Usable);
    static_assert([] {
        auto input = usableInput;
        input.qualificationExpired = false;
        return observeCapability(input).capability == Capability::Usable;
    }());
    static_assert([] {
        auto input = usableInput;
        input.modeEnabled = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Pending &&
               result.reason == CapabilityReason::ModeDisabled;
    }());
    static_assert([] {
        auto input = usableInput;
        input.identityCurrent = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Pending &&
               result.reason == CapabilityReason::AwaitingIdentity;
    }());
    static_assert([] {
        auto input = usableInput;
        input.qualificationExpired = false;
        input.candidatePublished = false;
        input.candidateResolvedForSupportHand = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Pending &&
               result.reason == CapabilityReason::AwaitingCandidate;
    }());
    static_assert([] {
        auto input = usableInput;
        input.candidatePublished = false;
        input.candidateResolvedForSupportHand = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason ==
                   CapabilityReason::CaptureUnavailableAfterQualification;
    }());
    static_assert([] {
        auto input = usableInput;
        input.candidateResolvedForSupportHand = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason ==
                   CapabilityReason::SupportTopologyUnavailable;
    }());
    static_assert([] {
        auto input = usableInput;
        input.weaponFamilyKnown = false;
        input.weaponFamilySupported = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason ==
                   CapabilityReason::UnclassifiedWeaponFamily;
    }());
    static_assert([] {
        auto input = usableInput;
        input.weaponFamilySupported = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason == CapabilityReason::UnsupportedWeaponFamily;
    }());
    static_assert([] {
        auto input = usableInput;
        input.canonicalAxesValid = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason == CapabilityReason::CanonicalAxesUnavailable;
    }());
    static_assert([] {
        auto input = usableInput;
        input.completeFingerPose = false;
        const auto result = observeCapability(input);
        return result.capability == Capability::Unavailable &&
               result.reason ==
                   CapabilityReason::CompleteFingerPoseUnavailable;
    }());
    static_assert(select(SelectionInput{
        .modeEnabled = true,
        .providerPartAuthorityActive = true,
        .dynamicHandoffCaptureEligible = true,
        .authoredCaptureEligible = true,
        .capability = Capability::Usable,
    }).selection == Selection::ProviderDynamic);
    static_assert([] {
        constexpr auto result = select(SelectionInput{
            .modeEnabled = true,
            .dynamicHandoffCaptureEligible = true,
            .authoredCaptureEligible = true,
            .capability = Capability::Usable,
        });
        return result.selection == Selection::DynamicHandoff &&
               result.reason ==
                   SelectionReason::AmbidextrousHandoffGrip;
    }());
    static_assert(select(SelectionInput{
        .modeEnabled = true,
        .authoredCaptureEligible = true,
        .capability = Capability::Unavailable,
    }).selection == Selection::Authored);
    static_assert(select(SelectionInput{
        .modeEnabled = false,
        .capability = Capability::Pending,
    }).selection == Selection::DynamicUnrestricted);
    static_assert(select(SelectionInput{
        .modeEnabled = true,
        .capability = Capability::Unavailable,
    }).selection == Selection::DynamicFallback);
    static_assert([] {
        constexpr auto result = select(SelectionInput{
            .modeEnabled = true,
            .capability = Capability::Pending,
        });
        return result.selection == Selection::Reject &&
               result.reason == SelectionReason::CapabilityPending;
    }());
    static_assert([] {
        constexpr auto result = select(SelectionInput{
            .modeEnabled = true,
            .capability = Capability::Usable,
        });
        return result.selection == Selection::Reject &&
               result.reason ==
                   SelectionReason::AuthoredActivationRejected;
    }());

    static_assert(captured(Selection::ProviderDynamic));
    static_assert(captured(Selection::DynamicHandoff));
    static_assert(captured(Selection::Authored));
    static_assert(captured(Selection::DynamicFallback));
    static_assert(captured(Selection::DynamicUnrestricted));
    static_assert(!captured(Selection::Reject));
    static_assert(!captured(Selection::Failure));
    static_assert(!captured(Selection::None));
    static_assert(kQualificationSeconds > 0.0f);
    static_assert(kUsableCandidateLossSeconds >= kQualificationSeconds);

    assert(std::abs(advanceContinuousEvidenceSeconds(
                        0.10f,
                        0.05f,
                        0.20f,
                        true) -
                    0.15f) < 0.000001f);
    assert(advanceContinuousEvidenceSeconds(
               0.19f,
               0.05f,
               0.20f,
               true) == 0.20f);
    assert(advanceContinuousEvidenceSeconds(
               0.19f,
               0.05f,
               0.20f,
               false) == 0.0f);
    assert(std::abs(advanceContinuousEvidenceSeconds(
                        0.10f,
                        (std::numeric_limits<float>::quiet_NaN)(),
                        0.20f,
                        true) -
                    0.10f) < 0.000001f);

    return 0;
}
