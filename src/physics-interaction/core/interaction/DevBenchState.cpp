// The devbench tool's state of hands and weapons: plain values copied on the
// frame thread, which the tool formats later on devbench's listener thread.
#include "devbench/RockDevBenchState.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/object/GrabTargetKind.h"
#include "rock_support/Fo4VrRuntime.h"

namespace rock
{
    namespace
    {
        const char* twoHandedStateName(TwoHandedState state) noexcept
        {
            switch (state) {
            case TwoHandedState::Inactive:
                return "Inactive";
            case TwoHandedState::Touching:
                return "Touching";
            case TwoHandedState::Gripping:
                return "Gripping";
            case TwoHandedState::PartCarry:
                return "PartCarry";
            case TwoHandedState::PrimaryOnly:
                return "PrimaryOnly";
            case TwoHandedState::SurfaceSupported:
                return "SurfaceSupported";
            }
            return "Unknown";
        }

        const char* handName(equipped_weapon_roster::Hand hand) noexcept
        {
            switch (hand) {
            case equipped_weapon_roster::Hand::Left:
                return "left";
            case equipped_weapon_roster::Hand::Right:
                return "right";
            case equipped_weapon_roster::Hand::None:
                break;
            }
            return "none";
        }

        std::string referenceName(RE::TESObjectREFR* ref)
        {
            const char* name = ref ? ref->GetDisplayFullName() : nullptr;
            return name ? std::string(name) : std::string();
        }

        /**
         * The weapon in a native equipment index, or none.
         */
        void captureWeapon(std::uint32_t index, devbench::WeaponState& out)
        {
            const auto* item = f4vr::getEquippedWeaponItem(index);
            auto* form = item ? item->item.object : nullptr;
            auto* weapon = form && form->formType == RE::ENUM_FORM_ID::kWEAP ? form->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon)
                return;
            out.formId = weapon->GetFormID();
            out.name = std::string(RE::TESFullName::GetFullName(*weapon, false));
        }
    }

    /**
     * Fills the interaction part of the devbench state: what each hand does and holds, and which equipped weapon
     * each hand fires. The held kinds follow the test-bench telemetry's: ROCK's own grab wins over the equipped
     * weapon's grips.
     */
    void PhysicsInteraction::captureDevBenchState(devbench::RockState& state) const
    {
        const auto& runtime = runtime_state::currentFrame();
        state.interaction = true;
        state.playerAvailable = runtime.playerAvailable;
        state.weaponDrawn = runtime.weaponDrawn;
        state.menuBlocking = runtime.localMenuBlocking;
        state.inputMenuBlocking = runtime.inputMenuBlocking;
        state.loading = runtime.localLoadingMenuOpen;

        const auto roster = equippedWeaponRoster();
        for (const bool isLeft : { true, false }) {
            const Hand& hand = isLeft ? _leftHand : _rightHand;
            auto& out = state.hands[isLeft ? 0 : 1];
            out.state = handStateName(hand.getState());
            const auto grips = roster.slotZeroGrips(isLeft);
            if (hand.isHolding()) {
                out.held = hand.isHoldingLooseWeapon() ? "loose-weapon" : "object";
                if (auto* ref = hand.getHeldRef()) {
                    out.heldFormId = ref->GetFormID();
                    out.heldName = referenceName(ref);
                }
            } else if (grips.firingGripActive || roster.slotOneHolds(isLeft)) {
                out.held = "equipped-weapon";
            } else if (grips.partGripActive) {
                out.held = "weapon-part";
            }
            if (hand.hasSelection()) {
                const auto& selection = hand.getSelection();
                if (selection.refr) {
                    out.selectedFormId = selection.refr->GetFormID();
                    out.selectedName = referenceName(selection.refr);
                }
                out.selectedKind = grab_target::name(selection.targetKind);
                out.selectionFar = selection.isFarSelection;
            }
        }

        state.twoHandedGrip = twoHandedStateName(_twoHandedGrip.getState());
        const auto& facts = roster.facts();
        captureWeapon(0, state.weapons[0]);
        if (state.weapons[0].formId && facts.slot0.holdsHand && !facts.slot0.firingGripDetached)
            state.weapons[0].hand = handName(facts.slot0.firingHand);
        if (facts.slot1.equipped) {
            captureWeapon(1, state.weapons[1]);
            if (facts.slot1.holdsHand)
                state.weapons[1].hand = handName(facts.slot1.hand);
        }
    }
}
