#include "physics-interaction/stash/ShoulderStashEligibility.h"

#include "physics-interaction/object/GrabTargetKind.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::shoulder_stash
{
    namespace
    {
        constexpr std::uint8_t kFo4BookCantBeTakenFlag = 1u << 1;
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
        }
        return "unknown";
    }

    bool isUntakeableBook(RE::TESBoundObject* baseForm) noexcept
    {
        auto* book = baseForm && baseForm->Is(RE::ENUM_FORM_ID::kBOOK) ? static_cast<RE::TESObjectBOOK*>(baseForm) : nullptr;
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
