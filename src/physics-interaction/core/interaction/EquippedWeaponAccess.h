// What the frame stages ask about the weapon in the player's hand: its form,
// instance data, native weapon data and interaction node, the reset of the
// firing-grip input state, and the records the equipped-weapon frame passes
// between its preparation and its result.
#pragma once

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cstdint>

namespace rock
{
    namespace physics_interaction_internal
    {
        inline void clearEquippedWeaponFiringGripInputState()
        {
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
            input_remap_runtime::setEquippedWeaponLeftHandFiringActive(false);
        }

        inline RE::TESObjectWEAP* currentEquippedWeaponForm()
        {
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            if (!weaponForm || weaponForm->formType != RE::ENUM_FORM_ID::kWEAP) {
                return nullptr;
            }

            return weaponForm->As<RE::TESObjectWEAP>();
        }

        inline RE::TBO_InstanceData* currentEquippedWeaponInstanceData(const RE::TESObjectWEAP* expectedWeapon)
        {
            auto* equipData = f4vr::getEquippedWeaponItem();
            if (!expectedWeapon || !equipData) {
                return nullptr;
            }

            auto* weaponForm = equipData->item.object;
            auto* equippedWeapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            return equippedWeapon == expectedWeapon ? equipData->item.instanceData.get() : nullptr;
        }

        inline std::uint32_t currentEquippedWeaponFormId()
        {
            const auto* weapon = currentEquippedWeaponForm();
            return weapon ? weapon->formID : 0;
        }

        inline bool currentEquippedWeaponOccupiesHand()
        {
            return fo4vr_actor_state_policy::equippedWeaponOccupiesHand(currentEquippedWeaponFormId() != 0,
                f4vr::getNativeWeaponState(f4vr::getPlayer()));
        }

        inline RE::EquippedWeaponData* getValidatedEquippedWeaponData()
        {
            auto* equipWeaponData = f4vr::getEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto vtable = *reinterpret_cast<const std::uintptr_t*>(equipWeaponData);
            if (vtable != f4vr::EquippedWeaponData_vtable.address()) {
                return nullptr;
            }
            return equipWeaponData;
        }

        inline RE::NiNode* resolveEquippedWeaponInteractionNodeDirect() { return f4vr::getWeaponNode(); }

        inline RE::NiNode* resolveEquippedWeaponInteractionNode()
        {
            /*
             * Weapon interaction uses the same first-person weapon root for all
             * equipped weapons. Generated collision scans every known package
             * candidate internally, so this handoff should not branch by weapon
             * type or create a separate melee-owned update path.
             */
            if (!runtime_state::currentFrame().weaponDrawn || !currentEquippedWeaponOccupiesHand()) {
                return nullptr;
            }

            return resolveEquippedWeaponInteractionNodeDirect();
        }
    }

    struct PhysicsInteraction::EquippedWeaponFramePreparation
    {
        // The equipped weapon's interaction node, valid for this frame only.
        RE::NiNode* weaponNode = nullptr;
        bool secondaryEquipped = false;
        bool secondaryLeft = false;
        bool secondaryRight = false;
        bool leftHandAvailableForAcquisition = false;
        bool rightHandAvailableForAcquisition = false;
        bool rightHandWeaponEquipped = false;
        bool retainedWeaponCollisionActive = false;
        // Leases taken before the collider update.
        bool rightHandWeaponAuthorityActive = false;
        bool leftSupportGripActive = false;
        bool rightPartGripActive = false;
    };

    struct PhysicsInteraction::EquippedWeaponFrameResult
    {
        bool rightHandWeaponAuthorityActive = false;
        bool leftSupportGripActive = false;
        bool rightPartGripActive = false;
    };
}
