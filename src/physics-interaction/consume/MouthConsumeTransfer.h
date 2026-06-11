#pragma once

#include "RE/Bethesda/TESForms.h"

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
        MissingBaseForm,
        UnsupportedBaseForm,
        ActivateRef,
        ActivateRefFailed,
        DrinkPotion,
        DrinkPotionFailed,
    };

    struct ConsumeInput
    {
        RE::TESObjectREFR* heldRef = nullptr;
    };

    struct ConsumeResult
    {
        bool attempted = false;
        bool success = false;
        ConsumeReason reason = ConsumeReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        RE::TESBoundObject* baseForm = nullptr;
    };

    [[nodiscard]] const char* consumeReasonName(ConsumeReason reason) noexcept;
    [[nodiscard]] ConsumeResult transferToPlayerConsume(const ConsumeInput& input) noexcept;
}
