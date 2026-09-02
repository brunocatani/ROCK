#pragma once

#include "physics-interaction/hand/Hand.h"

namespace rock::hand_grab_internal
{
    [[nodiscard]] bool tryGetGrabAuthorityBodyWorldTransform(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform);
}
