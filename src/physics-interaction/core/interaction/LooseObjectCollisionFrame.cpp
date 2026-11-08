// Experimental generated collision for held loose objects
// (Grabbing.GeneratedObjectCollision). Chooses which held references the
// object pool may admit; the pool itself (LooseWeaponCollision::Pool::Objects)
// builds the shapes, keeps them through a throw or drop and restores the
// native collision at rest. Turning the setting off restores every native
// shape at once.
#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock
{
    namespace
    {
        RE::TESObjectREFR* heldLooseObject(const Hand& hand)
        {
            if (!hand.isHolding() || hand.isHoldingLooseWeapon())
                return nullptr;
            // Connected and detached bodies stay with their native owner.
            const auto kind = hand.getSavedObjectState().targetKind;
            if (kind != grab_target::Kind::LooseObject && kind != grab_target::Kind::DynamicMovableStatic)
                return nullptr;
            auto* ref = hand.getHeldRef();
            if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kREFR)
                return nullptr; // Actors are never admitted.
            const auto* base = ref->GetObjectReference();
            // Weapons have their own pool; count-based ammunition carries a
            // separate root body that ROCK keeps on its visible round.
            if (!base || base->Is(RE::ENUM_FORM_ID::kWEAP) || base->Is(RE::ENUM_FORM_ID::kAMMO))
                return nullptr;
            return ref;
        }
    }

    void PhysicsInteraction::updateLooseObjectCollision(const PhysicsFrameContext& frame)
    {
        auto& gate = _generatedBodyStepDrive.callbackGate();
        if (!g_rockConfig.rockGeneratedObjectCollision) {
            if (!_looseObjectCollision.empty() && frame.worldReady && frame.hknpWorld && !frame.menuBlocked) {
                auto mutation = gate.pauseForMutation();
                _looseObjectCollision.clear(frame.hknpWorld);
            }
            return;
        }
        _looseObjectCollision.update(frame, heldLooseObject(_rightHand), heldLooseObject(_leftHand), gate);
    }
}
