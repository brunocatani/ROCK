#pragma once

/*
 * Shoulder stash policy keeps the small pure decision surfaces together:
 * eligibility, haptic feedback exposure, and user-facing notifications. The
 * runtime detector/transfer files own stateful work; this header owns the
 * compact rules future readers usually need side-by-side.
 */

#include "physics-interaction/feedback/HapticPolicy.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace RE
{
    class TESBoundObject;
    class TESObjectREFR;
}

namespace rock
{
    struct SavedObjectState;
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

namespace rock::shoulder_stash_notification_policy
{
    [[nodiscard]] inline std::string fallbackItemName(std::uint32_t formID)
    {
        if (formID == 0) {
            return "item";
        }

        char buffer[16]{};
        std::snprintf(buffer, sizeof(buffer), "%08X", formID);
        std::string name = "item ";
        name += buffer;
        return name;
    }

    [[nodiscard]] inline std::string formatCollectedNotification(std::string_view itemName, int count, std::uint32_t formID)
    {
        std::string message = "[ROCK] Collected ";
        message += itemName.empty() ? fallbackItemName(formID) : std::string(itemName);

        const int safeCount = (std::max)(1, count);
        if (safeCount > 1) {
            message += " x";
            message += std::to_string(safeCount);
        }

        return message;
    }
}
