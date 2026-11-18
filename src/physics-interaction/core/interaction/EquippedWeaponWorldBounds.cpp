// World bounds of the presented equipped weapons (see weapon/scene/WeaponWorldBounds.h).
// Runs at AfterWorldFinal, after every ROCK and FRIK weapon placement of the
// frame, on both native equipment slots and ROCK's equip copy.
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/native/NativeHandEquipment.h"
#include "physics-interaction/weapon/scene/WeaponWorldBounds.h"

namespace rock
{
    void PhysicsInteraction::refreshEquippedWeaponWorldBounds() noexcept
    {
        for (const auto& view : native_hand_equipment::equippedViews()) {
            if (view.weapon && view.root && !view.root->GetAppCulled())
                weapon_world_bounds::refresh(view.root);
        }
        weapon_world_bounds::refresh(_equipped.transition.presentedBridgeModel());
    }
}
