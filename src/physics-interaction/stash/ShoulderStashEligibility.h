#pragma once

#include "physics-interaction/grab/GrabConstraint.h"

#include "RE/Bethesda/TESForms.h"

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
}

namespace rock::shoulder_stash
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
        ActorBaseForm,
        NonPlayableBase,
        UntakeableBook,
    };

    struct EligibilityInput
    {
        bool enabled = true;
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
    [[nodiscard]] bool isUntakeableBook(RE::TESBoundObject* baseForm) noexcept;
    [[nodiscard]] EligibilityResult evaluateEligibility(const EligibilityInput& input) noexcept;
}
