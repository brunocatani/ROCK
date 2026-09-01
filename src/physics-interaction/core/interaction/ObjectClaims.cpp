#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Physics object claims and forced drops. Includes the contact-callback implementation (PhysicsInteractionContacts.inl).

namespace rock
{
#include "physics-interaction/core/PhysicsInteractionContacts.inl"
    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_claims.mutex);
        const auto it = _claims.owned.find(ref->GetFormID());
        return it != _claims.owned.end() && it->second != 0;
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_claims.mutex);
        const auto it = _claims.owned.find(ref->GetFormID());
        return it != _claims.owned.end() && (it->second & claimOwnerBit(owner)) != 0;
    }

    void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_claims.mutex);
        auto& ownerMask = _claims.owned[formID];
        const auto previousMask = ownerMask;
        ownerMask |= claimOwnerBit(owner);
        ROCK_LOG_DEBUG(Hand,
            "Claimed object: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} owners={}",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask,
            ownerMask,
            claimOwnerCount(ownerMask));
    }

    void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_claims.mutex);
        auto it = _claims.owned.find(formID);
        if (it == _claims.owned.end()) {
            return;
        }

        const auto previousMask = it->second;
        it->second &= ~claimOwnerBit(owner);
        if (it->second != 0) {
            ROCK_LOG_DEBUG(Hand,
                "Released object claim: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} ownersRemaining={}",
                formID,
                static_cast<std::uint32_t>(owner),
                previousMask,
                it->second,
                claimOwnerCount(it->second));
            return;
        }

        _claims.owned.erase(it);
        ROCK_LOG_DEBUG(Hand,
            "Released object: formID={:08X} owner={} mask=0x{:02X}->0x00",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask);
    }

    void PhysicsInteraction::releaseAllObjects()
    {
        std::scoped_lock lock(_claims.mutex);
        if (!_claims.owned.empty()) {
            ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _claims.owned.size());
            _claims.owned.clear();
        }

        (void)frik_visual_authority::blockOffHandWeaponGripping("ROCK_Physics", false);
    }

    void PhysicsInteraction::forceDropHeldObject(bool isLeft)
    {
        auto& hand = isLeft ? _leftHand : _rightHand;
        if (!hand.isHolding())
            return;

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no bhkWorld available");
            return;
        }
        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no hknpWorld available");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        ROCK_LOG_INFO(Hand, "forceDropHeldObject: {} hand dropping {}", isLeft ? "Left" : "Right", heldRef ? heldRef->GetFormID() : 0);

        hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
        if (heldRef)
            releaseObject(heldRef, claimOwnerForHand(isLeft));
        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
    }
}
