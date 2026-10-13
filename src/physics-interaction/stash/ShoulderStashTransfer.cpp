#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "api/StashConversionService.h"

#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <utility>

namespace rock::shoulder_stash
{
    namespace
    {
        constexpr std::ptrdiff_t kExtraCountCountOffset = 0x18;
    }

    const char* transferReasonName(TransferReason reason) noexcept
    {
        switch (reason) {
        case TransferReason::MissingRef:
            return "missing-ref";
        case TransferReason::MissingPlayer:
            return "missing-player";
        case TransferReason::MissingBaseForm:
            return "missing-base-form";
        case TransferReason::ConsumerConversion:
            return "consumer-conversion";
        case TransferReason::ActivateRef:
            return "activate-ref";
        default:
            return "not-attempted";
        }
    }

    std::int32_t resolveReferenceStackCount(RE::TESObjectREFR* refr) noexcept
    {
        if (!refr || !refr->extraList) {
            return 1;
        }

        auto* extraData = refr->extraList->GetByType(RE::EXTRA_DATA_TYPE::kCount);
        if (!extraData) {
            return 1;
        }

        const auto* raw = reinterpret_cast<const std::uint8_t*>(extraData);
        std::uint16_t count = 0;
        std::memcpy(&count, raw + kExtraCountCountOffset, sizeof(count));
        return (std::max<std::int32_t>)(1, static_cast<std::int32_t>(count));
    }

    TransferResult transferToPlayerInventory(TransferInput input) noexcept
    {
        TransferResult result{};
        auto* heldRef = input.heldRef.get();
        result.untransferredRef = std::move(input.heldRef);
        if (!heldRef) {
            result.reason = TransferReason::MissingRef;
            return result;
        }

        result.formID = heldRef->GetFormID();
        result.count = resolveReferenceStackCount(heldRef);
        result.baseForm = heldRef->GetObjectReference();
        if (!result.baseForm) {
            result.reason = TransferReason::MissingBaseForm;
            return result;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = TransferReason::MissingPlayer;
            return result;
        }

        result.attempted = true;
        const auto conversion=stash_conversion::convert(result.formID,result.baseForm->formID);
        if(conversion.decision!=api::stash_conversion::Decision::Pass) {
            result.reason=TransferReason::ConsumerConversion;
            result.success=conversion.decision==api::stash_conversion::Decision::Consumed;
            if(result.success) {
                result.count=static_cast<std::int32_t>(conversion.count);
                result.baseForm=RE::TESForm::GetFormByID<RE::TESBoundObject>(conversion.inventoryForm);
                result.untransferredRef.reset();
            }
            return result;
        }
        // Collectibles such as magazines, notes, and holotapes need native activation
        // so their scripts, perk grants, and terminal/holotape behavior run normally.
        result.success = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        if (result.success) {
            result.untransferredRef.reset();
        }
        result.reason = TransferReason::ActivateRef;
        return result;
    }
}
