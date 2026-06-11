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
        case ConsumeReason::MissingEquipManager:
            return "missing-equip-manager";
        case ConsumeReason::DeletedOrDisabled:
            return "deleted-or-disabled";
        case ConsumeReason::PlayerRef:
            return "player-ref";
        case ConsumeReason::MissingBaseForm:
            return "missing-base-form";
        case ConsumeReason::NonPlayableBase:
            return "non-playable-base";
        case ConsumeReason::UnsupportedBaseForm:
            return "unsupported-base-form";
        case ConsumeReason::PoisonBlockedByConfig:
            return "poison-blocked-by-config";
        case ConsumeReason::StackedReferenceUnsupported:
            return "stacked-reference-unsupported";
        case ConsumeReason::ActivateRef:
            return "activate-ref";
        case ConsumeReason::ActivateRefThenUseObject:
            return "activate-ref-use-object";
        case ConsumeReason::ActivateRefFailed:
            return "activate-ref-failed";
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
        if (heldRef->IsDeleted() || heldRef->IsDisabled()) {
            result.reason = ConsumeReason::DeletedOrDisabled;
            return result;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = ConsumeReason::MissingPlayer;
            return result;
        }

        if (heldRef == player) {
            result.reason = ConsumeReason::PlayerRef;
            return result;
        }

        result.baseForm = heldRef->GetObjectReference();
        if (!result.baseForm) {
            result.reason = ConsumeReason::MissingBaseForm;
            return result;
        }

        if (!result.baseForm->GetPlayable(result.baseForm->GetBaseInstanceData())) {
            result.reason = ConsumeReason::NonPlayableBase;
            return result;
        }

        if (!isSupportedConsumableBaseForm(result.baseForm)) {
            result.reason = ConsumeReason::UnsupportedBaseForm;
            return result;
        }

        if (!input.allowPoison && result.baseForm->Is(RE::ENUM_FORM_ID::kALCH)) {
            auto* alchemyItem = static_cast<RE::AlchemyItem*>(result.baseForm);
            if (alchemyItem->IsPoison()) {
                result.reason = ConsumeReason::PoisonBlockedByConfig;
                return result;
            }
        }

        if (result.count > 1) {
            result.reason = ConsumeReason::StackedReferenceUnsupported;
            return result;
        }

        const bool useAsInventoryObject = result.baseForm->Is(RE::ENUM_FORM_ID::kALCH);
        auto* equipManager = useAsInventoryObject ? RE::ActorEquipManager::GetSingleton() : nullptr;
        if (useAsInventoryObject && !equipManager) {
            result.reason = ConsumeReason::MissingEquipManager;
            return result;
        }

        result.attempted = true;
        const bool activated = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        if (!activated) {
            result.reason = ConsumeReason::ActivateRefFailed;
            return result;
        }

        if (useAsInventoryObject) {
            RE::BGSObjectInstance objectInstance(result.baseForm, nullptr);
            // FO4VR Ghidra verification: ActorEquipManager::EquipObject at 0x140e6fea0
            // dispatches consumables through UseObject at 0x140e712d0 when forceEquip is false.
            // A null instance pointer lets native code resolve the stack from player inventory
            // after ActivateRef has transferred the loose reference.
            // The wrapper can return false for that use path, so activation success is the
            // transfer boundary and the use call is a best-effort inventory consume dispatch.
            static_cast<void>(equipManager->EquipObject(player, objectInstance, 0, 1, nullptr, false, false, true, false, false));
            result.success = true;
            result.reason = ConsumeReason::ActivateRefThenUseObject;
            return result;
        }

        result.success = true;
        result.reason = ConsumeReason::ActivateRef;
        return result;
    }
}
