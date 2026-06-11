#include "physics-interaction/consume/MouthConsumeTransfer.h"

#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/MagicItems.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::mouth_consume
{
    const char* consumeReasonName(ConsumeReason reason) noexcept
    {
        switch (reason) {
        case ConsumeReason::MissingRef:
            return "missing-ref";
        case ConsumeReason::MissingPlayer:
            return "missing-player";
        case ConsumeReason::MissingBaseForm:
            return "missing-base-form";
        case ConsumeReason::UnsupportedBaseForm:
            return "unsupported-base-form";
        case ConsumeReason::ActivateRef:
            return "activate-ref";
        case ConsumeReason::ActivateRefFailed:
            return "activate-ref-failed";
        case ConsumeReason::DrinkPotion:
            return "drink-potion";
        case ConsumeReason::DrinkPotionFailed:
            return "drink-potion-failed";
        default:
            return "not-attempted";
        }
    }

    ConsumeResult transferToPlayerConsume(const ConsumeInput& input) noexcept
    {
        ConsumeResult result{};
        auto* heldRef = input.heldRef;
        if (!heldRef) {
            result.reason = ConsumeReason::MissingRef;
            return result;
        }

        result.formID = heldRef->GetFormID();
        result.count = shoulder_stash::resolveReferenceStackCount(heldRef);
        result.baseForm = heldRef->GetObjectReference();
        if (!result.baseForm) {
            result.reason = ConsumeReason::MissingBaseForm;
            return result;
        }

        if (!isSupportedConsumableBaseForm(result.baseForm)) {
            result.reason = ConsumeReason::UnsupportedBaseForm;
            return result;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = ConsumeReason::MissingPlayer;
            return result;
        }

        result.attempted = true;
        const bool activated = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        if (!activated) {
            result.reason = ConsumeReason::ActivateRefFailed;
            return result;
        }

        if (result.baseForm->Is(RE::ENUM_FORM_ID::kALCH)) {
            auto* alchemyItem = static_cast<RE::AlchemyItem*>(result.baseForm);
            if (player->DrinkPotion(alchemyItem, 0)) {
                result.success = true;
                result.reason = ConsumeReason::DrinkPotion;
            } else {
                result.reason = ConsumeReason::DrinkPotionFailed;
            }
            return result;
        }

        result.success = true;
        result.reason = ConsumeReason::ActivateRef;
        return result;
    }
}
