#include "physics-interaction/core/PhysicsInteraction.h"

namespace rock {
    bool PhysicsInteraction::prepareLooseWeaponRelease(RE::TESObjectREFR* reference,
        RE::hknpWorld* world, bool physicalDrop)
    {
        return _looseWeaponCollision.prepareRelease(reference, world,
            _generatedBodyStepDrive.callbackGate(), physicalDrop);
    }
}
