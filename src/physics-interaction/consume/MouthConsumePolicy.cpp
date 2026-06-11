#include "physics-interaction/consume/MouthConsumePolicy.h"

#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/object/GrabTargetKind.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"

#include "RE/Bethesda/MagicItems.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::mouth_consume
{
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
        case EligibilityReason::NonPlayableBase:
            return "non-playable-base";
        case EligibilityReason::UnsupportedBaseForm:
            return "unsupported-base-form";
        case EligibilityReason::PoisonBlockedByConfig:
            return "poison-blocked-by-config";
        case EligibilityReason::StackedReferenceUnsupported:
            return "stacked-reference-unsupported";
        }
        return "unknown";
    }

    bool isSupportedConsumableBaseForm(RE::TESBoundObject* baseForm) noexcept
    {
        return baseForm && baseForm->Is(RE::ENUM_FORM_ID::kALCH);
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

        if (!baseForm->GetPlayable(baseForm->GetBaseInstanceData())) {
            result.reason = EligibilityReason::NonPlayableBase;
            return result;
        }

        if (!isSupportedConsumableBaseForm(baseForm)) {
            result.reason = EligibilityReason::UnsupportedBaseForm;
            return result;
        }

        if (!input.allowPoison && baseForm->Is(RE::ENUM_FORM_ID::kALCH)) {
            auto* alchemyItem = static_cast<RE::AlchemyItem*>(baseForm);
            if (alchemyItem->IsPoison()) {
                result.reason = EligibilityReason::PoisonBlockedByConfig;
                return result;
            }
        }

        if (shoulder_stash::resolveReferenceStackCount(heldRef) > 1) {
            result.reason = EligibilityReason::StackedReferenceUnsupported;
            return result;
        }

        result.eligible = true;
        result.reason = EligibilityReason::Eligible;
        return result;
    }
}
