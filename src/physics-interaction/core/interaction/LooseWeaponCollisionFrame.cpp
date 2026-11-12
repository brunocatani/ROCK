// Called before a hand releases a loose weapon, while the reference and its
// native bodies still exist. A physical drop restores the weapon's native
// collision and keeps its rest watch; any other release frees ROCK's collision
// state for it (LooseWeaponCollision). A failed restore keeps the grab.
#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock
{
    bool PhysicsInteraction::prepareLooseWeaponRelease(RE::TESObjectREFR* reference, RE::hknpWorld* world,
        bool physicalDrop)
    {
        return _looseWeaponCollision.prepareRelease(reference, world, _generatedBodyStepDrive.callbackGate(),
            physicalDrop);
    }
}
