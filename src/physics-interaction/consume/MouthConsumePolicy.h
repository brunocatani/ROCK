#pragma once

#include "physics-interaction/feedback/HapticPolicy.h"

#include <cstdint>

namespace RE
{
    class TESBoundObject;
    class TESObjectREFR;
}

namespace rock
{
    struct SavedObjectState;
}

namespace rock::mouth_consume
{
    enum class EligibilityReason : std::uint8_t
    {
        Eligible = 0,
        DisabledByConfig,
        MissingHeldRef,
        DeletedOrDisabled,
        PlayerRef,
        SharedHeldObject,
        MissingSavedState,
        NonLooseRockTarget,
        MissingBaseForm,
        NonPlayableBase,
        UnsupportedBaseForm,
        PoisonBlockedByConfig,
        UntakeableBook,
        StackedReferenceUnsupported,
    };

    struct EligibilityInput
    {
        bool enabled = true;
        bool allowPoison = false;
        bool peerHoldingSameObject = false;
        RE::TESObjectREFR* heldRef = nullptr;
        const SavedObjectState* savedState = nullptr;
    };

    struct EligibilityResult
    {
        bool eligible = false;
        EligibilityReason reason = EligibilityReason::MissingHeldRef;
        RE::TESBoundObject* baseForm = nullptr;
    };

    [[nodiscard]] const char* eligibilityReasonName(EligibilityReason reason) noexcept;
    [[nodiscard]] bool isSupportedConsumableBaseForm(RE::TESBoundObject* baseForm) noexcept;
    [[nodiscard]] EligibilityResult evaluateEligibility(const EligibilityInput& input) noexcept;
}

namespace rock::mouth_consume_haptic_policy
{
    using CandidatePulseConfig = candidate_haptic_policy::CandidatePulseConfig;

    [[nodiscard]] inline float computeCandidatePulseIntensity(float confidence, const CandidatePulseConfig& config) noexcept
    {
        return candidate_haptic_policy::computeCandidatePulseIntensity(confidence, config);
    }
}
