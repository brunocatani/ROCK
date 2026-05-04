#pragma once

#include "RE/Bethesda/BSHavok.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock
{
    /*
     * A physics frame reads the same world pointers, hand transforms, hand
     * nodes, and derived hand vectors from multiple subsystems. Building this
     * bundle once at the top of the frame keeps those reads coherent and makes
     * later state-machine and Havok-runtime work pass explicit frame data
     * instead of re-querying global FRIK/FO4VR state in each substep.
     */
    struct HandFrameInput
    {
        bool isLeft = false;
        RE::NiTransform rawHandWorld{};
        RE::NiNode* handNode = nullptr;
        RE::NiPoint3 grabAnchorWorld{};
        RE::NiPoint3 palmNormalWorld{};
        RE::NiPoint3 pointingWorld{};
        bool disabled = false;
    };

    struct PhysicsFrameContext
    {
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpWorld* hknpWorld = nullptr;
        float deltaSeconds = 1.0f / 90.0f;
        bool worldReady = false;
        bool menuBlocked = false;
        bool reloadBoundaryActive = false;
        HandFrameInput right{};
        HandFrameInput left{};
    };
}
