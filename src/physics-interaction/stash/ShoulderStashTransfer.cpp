#include "physics-interaction/stash/ShoulderStashTransfer.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

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
        case TransferReason::ActivateRef:
            return "activate-ref";
        case TransferReason::ActorPickUpObjectBook:
            return "actor-pickup-object-book";
        case TransferReason::ActorPickUpObjectNote:
            return "actor-pickup-object-note";
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

    TransferResult transferToPlayerInventory(const TransferInput& input) noexcept
    {
        TransferResult result{};
        auto* heldRef = input.heldRef;
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
        if (input.skipActivateBooks && result.baseForm->Is(RE::ENUM_FORM_ID::kBOOK)) {
            player->PickUpObject(heldRef, result.count, input.playPickupSounds);
            result.success = true;
            result.reason = TransferReason::ActorPickUpObjectBook;
            return result;
        }

        if (input.skipActivateNotes && result.baseForm->Is(RE::ENUM_FORM_ID::kNOTE)) {
            player->PickUpObject(heldRef, result.count, input.playPickupSounds);
            result.success = true;
            result.reason = TransferReason::ActorPickUpObjectNote;
            return result;
        }

        result.success = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        result.reason = TransferReason::ActivateRef;
        return result;
    }
}
