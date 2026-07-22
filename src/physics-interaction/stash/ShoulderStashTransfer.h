#pragma once

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiSmartPointer.h"

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
    };

    struct TransferInput
    {
        RE::NiPointer<RE::TESObjectREFR> heldRef{};
    };

    struct TransferResult
    {
        bool attempted = false;
        bool success = false;
        TransferReason reason = TransferReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        RE::TESBoundObject* baseForm = nullptr;
        RE::NiPointer<RE::TESObjectREFR> untransferredRef{};
    };

    [[nodiscard]] const char* transferReasonName(TransferReason reason) noexcept;
    [[nodiscard]] std::int32_t resolveReferenceStackCount(RE::TESObjectREFR* refr) noexcept;
    [[nodiscard]] TransferResult transferToPlayerInventory(TransferInput input) noexcept;
}
