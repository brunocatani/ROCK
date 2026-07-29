#pragma once

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
}

namespace rock::mouth_consume
{
    enum class ConsumeReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingRef,
        MissingPlayer,
        MissingEquipManager,
        DeletedOrDisabled,
        PlayerRef,
        MissingBaseForm,
        NonPlayableBase,
        UnsupportedBaseForm,
        PoisonBlockedByConfig,
        StackedReferenceUnsupported,
        ActivateRef,
        ActivateRefThenUseObject,
        ActivateRefFailed,
    };

    struct ConsumeInput
    {
        RE::NiPointer<RE::TESObjectREFR> heldRef{};
        bool allowPoison = false;
    };

    struct ConsumeResult
    {
        bool attempted = false;
        bool success = false;
        ConsumeReason reason = ConsumeReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        RE::TESBoundObject* baseForm = nullptr;
        RE::NiPointer<RE::TESObjectREFR> untransferredRef{};
    };

    [[nodiscard]] const char* consumeReasonName(ConsumeReason reason) noexcept;
    [[nodiscard]] ConsumeResult transferToPlayerConsume(ConsumeInput input) noexcept;
}
