#pragma once

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace rock::runtime_state
{
    struct PlayerSpaceFrame
    {
        bool valid = false;
        bool moving = false;
        const char* source = "none";
        RE::NiTransform world{};
        RE::NiPoint3 deltaGameUnits{};
    };

    struct RuntimeFrameInput
    {
        bool menuInputBlocking = false;
        bool visualAuthorityAvailable = false;
        bool visualSkeletonReadyHint = false;
        bool compatibilityConfigBlocking = false;
    };

    struct RuntimeFrameSnapshot
    {
        std::uint64_t frameIndex = 0;
        float deltaSeconds = 1.0f / 90.0f;
        bool playerAvailable = false;
        bool weaponDrawn = false;
        bool localMenuBlocking = false;
        bool localLoadingMenuOpen = false;
        bool localGameStopped = false;
        bool inputMenuBlocking = false;
        bool compatibilityConfigBlocking = false;
        bool visualAuthorityAvailable = false;
        bool visualSkeletonReadyHint = false;
        bool localSkeletonReady = false;
        bool localSkeletonRootAttached = false;
        bool localSkeletonRequiredHandBonesReady = false;
        bool localSkeletonRequiredBodyBonesReady = false;
        PlayerSpaceFrame playerSpace{};
    };

    void initialize();
    void resetTransientState();
    void updateFrame(const RuntimeFrameInput& input);

    [[nodiscard]] const RuntimeFrameSnapshot& currentFrame();
    [[nodiscard]] bool isLocalSkeletonReady();
    [[nodiscard]] bool isPhysicsMenuBlocked();
    [[nodiscard]] bool isCompatibilityConfigBlocked();
    [[nodiscard]] float deltaSeconds();
}
