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
        // An activator is not an inventory item; activating one runs whatever
        // it does (a movable prop such as a pre-war mug starts a quest scene).
        // Loot activators (see lootActivatorScript) are the one exception.
        ActivatorBaseForm,
    };

    struct EligibilityInput
    {
        bool enabled = true;
        bool peerHoldingSameObject = false;
        RE::TESObjectREFR* heldRef = nullptr;
        const SavedObjectState* savedState = nullptr;
        // Bound loot script of an activator, from resolveLootActivatorScript.
        const char* lootActivatorScript = nullptr;
    };

    struct EligibilityResult
    {
        bool eligible = false;
        EligibilityReason reason = EligibilityReason::MissingHeldRef;
        RE::TESBoundObject* baseForm = nullptr;
        const char* lootActivatorScript = nullptr;
    };

    /*
     * A loot activator is a world prop whose activation hands the activating
     * actor an item and removes the prop: the vanilla caps stash tin and the
     * placed caps of the bottlecap mine and Money Shot use bobbyPinBoxSCRIPT
     * (leveled caps, then disable and delete); DefaultAddItemOnActivate with
     * bDisableWhenDone set does the same for a single item. Stashing one is
     * taking it, so the shoulder stash activates it like a pickup. The answer
     * is fixed for one grab, so the frame loop asks the VM once per grab.
     */
    struct LootActivatorMemo
    {
        std::uint64_t grabIdentity = 0;
        std::uint32_t formID = 0;
        const char* script = nullptr;
    };

    // Game thread. Returns the bound loot script name, or null for any other
    // reference, including an activator whose scripts cannot be read.
    [[nodiscard]] const char* resolveLootActivatorScript(LootActivatorMemo& memo, std::uint64_t grabIdentity,
        RE::TESObjectREFR* heldRef);

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

    [[nodiscard]] inline std::string formatCollectedNotification(std::string_view itemName, int count,
        std::uint32_t formID)
    {
        std::string message = "Collected ";
        message += itemName.empty() ? fallbackItemName(formID) : std::string(itemName);

        const int safeCount = (std::max)(1, count);
        if (safeCount > 1) {
            message += " x";
            message += std::to_string(safeCount);
        }

        return message;
    }

    // An equipped weapon stashed at the shoulder returns to the inventory;
    // Stashed keeps it distinct from a picked-up item's Collected.
    [[nodiscard]] inline std::string formatStashedNotification(std::string_view itemName, std::uint32_t formID)
    {
        std::string message = "Stashed ";
        message += itemName.empty() ? fallbackItemName(formID) : std::string(itemName);
        return message;
    }
}
