#include "physics-interaction/stash/ShoulderStashPolicy.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/native/script/NativeScriptBinding.h"
#include "physics-interaction/object/GrabTargetKind.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::shoulder_stash
{
    namespace
    {
        constexpr std::uint8_t kFo4BookCantBeTakenFlag = 1u << 1;
        // Vanilla loot scripts; Fallout4.esm binds the first to the caps stash
        // tin (0003000C), BottleCapMinePlacedCaps and MoneyShotPlacedCaps.
        constexpr const char* kLootScriptTakeAll = "bobbyPinBoxSCRIPT";
        constexpr const char* kLootScriptAddItem = "DefaultAddItemOnActivate";
    }

    const char* eligibilityReasonName(EligibilityReason reason) noexcept
    {
        switch (reason) {
        case EligibilityReason::Eligible:
            return "eligible";
        case EligibilityReason::DisabledByConfig:
            return "disabled-by-config";
        case EligibilityReason::MissingHeldRef:
            return "missing-held-ref";
        case EligibilityReason::DeletedOrDisabled:
            return "deleted-or-disabled";
        case EligibilityReason::PlayerRef:
            return "player-ref";
        case EligibilityReason::SharedHeldObject:
            return "shared-held-object";
        case EligibilityReason::MissingSavedState:
            return "missing-saved-state";
        case EligibilityReason::NonLooseRockTarget:
            return "non-loose-rock-target";
        case EligibilityReason::MissingBaseForm:
            return "missing-base-form";
        case EligibilityReason::ActorBaseForm:
            return "actor-base-form";
        case EligibilityReason::NonPlayableBase:
            return "non-playable-base";
        case EligibilityReason::UntakeableBook:
            return "untakeable-book";
        case EligibilityReason::ActivatorBaseForm:
            return "activator-base-form";
        }
        return "unknown";
    }

    const char* resolveLootActivatorScript(LootActivatorMemo& memo, std::uint64_t grabIdentity,
        RE::TESObjectREFR* heldRef)
    {
        auto* baseForm = heldRef ? heldRef->GetObjectReference() : nullptr;
        if (!baseForm || !baseForm->Is(RE::ENUM_FORM_ID::kACTI) || grabIdentity == 0) {
            return nullptr;
        }
        const auto formID = heldRef->GetFormID();
        if (memo.grabIdentity == grabIdentity && memo.formID == formID) {
            return memo.script;
        }

        memo = { .grabIdentity = grabIdentity, .formID = formID, .script = nullptr };
        const char* failedStage = nullptr;
        const auto takeScript = native_script_binding::findBoundScript(heldRef, kLootScriptTakeAll);
        if (takeScript.script) {
            memo.script = kLootScriptTakeAll;
        } else if (!takeScript.failedStage) {
            const auto addItem = native_script_binding::findBoundScript(heldRef, kLootScriptAddItem);
            bool disablesWhenDone = false;
            if (addItem.script &&
                native_script_binding::readBoolProperty(addItem, "bDisableWhenDone", disablesWhenDone, failedStage) &&
                disablesWhenDone) {
                memo.script = kLootScriptAddItem;
            }
            if (!failedStage) {
                failedStage = addItem.failedStage;
            }
        } else {
            failedStage = takeScript.failedStage;
        }

        ROCK_LOG_DEBUG(Hand, "Shoulder stash activator ref={:08X} base={:08X} lootScript={} stage={}", formID,
            baseForm->GetFormID(), memo.script ? memo.script : "none", failedStage ? failedStage : "none");
        return memo.script;
    }

    bool isUntakeableBook(RE::TESBoundObject* baseForm) noexcept
    {
        auto* book =
            baseForm && baseForm->Is(RE::ENUM_FORM_ID::kBOOK) ? static_cast<RE::TESObjectBOOK*>(baseForm) : nullptr;
        return book && ((static_cast<std::uint8_t>(book->data.flags) & kFo4BookCantBeTakenFlag) != 0);
    }

    EligibilityResult evaluateEligibility(const EligibilityInput& input) noexcept
    {
        EligibilityResult result{};
        if (!input.enabled) {
            result.reason = EligibilityReason::DisabledByConfig;
            return result;
        }

        auto* heldRef = input.heldRef;
        if (!heldRef) {
            result.reason = EligibilityReason::MissingHeldRef;
            return result;
        }

        if (heldRef->IsDeleted() || heldRef->IsDisabled()) {
            result.reason = EligibilityReason::DeletedOrDisabled;
            return result;
        }

        if (heldRef == RE::PlayerCharacter::GetSingleton()) {
            result.reason = EligibilityReason::PlayerRef;
            return result;
        }

        if (input.peerHoldingSameObject) {
            result.reason = EligibilityReason::SharedHeldObject;
            return result;
        }

        if (!input.savedState || !input.savedState->isValid()) {
            result.reason = EligibilityReason::MissingSavedState;
            return result;
        }

        if (input.savedState->targetKind != grab_target::Kind::LooseObject) {
            result.reason = EligibilityReason::NonLooseRockTarget;
            return result;
        }

        auto* baseForm = heldRef->GetObjectReference();
        result.baseForm = baseForm;
        if (!baseForm) {
            result.reason = EligibilityReason::MissingBaseForm;
            return result;
        }

        if (baseForm->Is(RE::ENUM_FORM_ID::kNPC_)) {
            result.reason = EligibilityReason::ActorBaseForm;
            return result;
        }

        if (baseForm->Is(RE::ENUM_FORM_ID::kACTI)) {
            // Activators carry no playable flag; a loot activator is taken by
            // activation, every other activator stays refused.
            if (!input.lootActivatorScript) {
                result.reason = EligibilityReason::ActivatorBaseForm;
                return result;
            }
            result.lootActivatorScript = input.lootActivatorScript;
            result.eligible = true;
            result.reason = EligibilityReason::Eligible;
            return result;
        }

        if (!baseForm->GetPlayable(baseForm->GetBaseInstanceData())) {
            result.reason = EligibilityReason::NonPlayableBase;
            return result;
        }

        if (isUntakeableBook(baseForm)) {
            result.reason = EligibilityReason::UntakeableBook;
            return result;
        }

        result.eligible = true;
        result.reason = EligibilityReason::Eligible;
        return result;
    }
}
