#pragma once

#include "RE/Bethesda/BSHavok.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace rock
{
    /*
     * A physics frame reads the same world pointers, hand transforms, hand
     * nodes, and derived hand vectors from multiple subsystems. Building this
     * bundle once at the top of the frame keeps those reads coherent and makes
     * later state-machine and Havok-runtime work pass explicit frame data
     * instead of re-querying global skeleton and FO4VR state in each substep.
     */
    struct HandFrameInput
    {
        bool isLeft = false;
        RE::NiTransform rawHandWorld{};
        RE::NiNode* handNode = nullptr;
        RE::NiPoint3 grabAnchorWorld{};
        RE::NiPoint3 palmNormalWorld{};
        RE::NiPoint3 pointingWorld{};
        RE::NiPoint3 closeSelectionDirectionWorld{};
        RE::NiPoint3 farSelectionDirectionWorld{};
        RE::NiPoint3 pinchDirectionWorld{};
        RE::NiPoint3 thumbPadWorld{};
        RE::NiPoint3 indexPadWorld{};
        RE::NiPoint3 pinchPocketWorld{};
        bool hasPinchPocketWorld = false;
        bool disabled = false;
    };

    struct TrackedNodeFrame
    {
        RE::NiTransform world{};
        bool valid = false;
    };

    struct PhysicsFrameContext
    {
        std::uint64_t gameFrameIndex = 0;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpWorld* hknpWorld = nullptr;
        float deltaSeconds = 1.0f / 90.0f;
        bool worldReady = false;
        bool menuBlocked = false;
        bool reloadBoundaryActive = false;
        bool hasHmdFrame = false;
        RE::NiPoint3 hmdPositionWorld{};
        RE::NiPoint3 hmdForwardWorld{};
        TrackedNodeFrame rightWand{};
        TrackedNodeFrame leftWand{};
        TrackedNodeFrame rightWeaponDriver{};
        TrackedNodeFrame leftWeaponDriver{};
        HandFrameInput right{};
        HandFrameInput left{};
    };
}
