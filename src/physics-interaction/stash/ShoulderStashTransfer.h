#pragma once

#include "RE/Bethesda/TESForms.h"

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
}

namespace rock::shoulder_stash
{
    enum class TransferReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingRef,
        MissingPlayer,
        MissingBaseForm,
        ActivateRef,
        ActorPickUpObjectBook,
        ActorPickUpObjectNote,
    };

    struct TransferInput
    {
        RE::TESObjectREFR* heldRef = nullptr;
        bool skipActivateBooks = false;
        bool skipActivateNotes = false;
        bool playPickupSounds = true;
    };

    struct TransferResult
    {
        bool attempted = false;
        bool success = false;
        TransferReason reason = TransferReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        RE::TESBoundObject* baseForm = nullptr;
    };

    [[nodiscard]] const char* transferReasonName(TransferReason reason) noexcept;
    [[nodiscard]] std::int32_t resolveReferenceStackCount(RE::TESObjectREFR* refr) noexcept;
    [[nodiscard]] TransferResult transferToPlayerInventory(const TransferInput& input) noexcept;
}
