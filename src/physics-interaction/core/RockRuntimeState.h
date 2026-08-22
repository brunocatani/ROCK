#pragma once

#include "physics-interaction/timing/GameFrameTimingPolicy.h"

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
        bool visualAuthorityAvailable = false;
        bool visualSkeletonReadyHint = false;
        bool compatibilityConfigBlocking = false;
    };

    struct RuntimeFrameSnapshot
    {
        std::uint64_t frameIndex = 0;
        /*
         * Authoritative game-frame clock snapshot for this frame. New and
         * migrated consumers read this; deltaSeconds below is the legacy
         * sanitized value that still fabricates a nominal-rate delta for
         * unmeasurable frames and is scheduled for removal once every
         * consumer is migrated.
         */
        game_frame_timing_policy::GameFrameTiming timing{};
        float deltaSeconds = 0.0f;
        bool playerAvailable = false;
        bool weaponDrawn = false;
        bool localMenuBlocking = false;
        bool localScopeMenuOpen = false;
        bool localLoadingMenuOpen = false;
        bool localGameStopped = false;
        bool inputMenuBlocking = false;
        bool compatibilityConfigBlocking = false;
        bool visualAuthorityAvailable = false;
        bool visualSkeletonReadyHint = false;
        bool localSkeletonReady = false;
        bool localSkeletonRootAttached = false;
        bool localSkeletonRequiredHandBonesReady = false;
        PlayerSpaceFrame playerSpace{};
    };

    void initialize();
    void resetTransientState();

    /*
     * Creates the game-frame timing identity for this frame and samples menu
     * state once. Call at the top of the game-loop hook, after the original
     * game call returns and before any animation-phase dispatch; every phase
     * and updateFrame() then share this one snapshot.
     */
    const game_frame_timing_policy::GameFrameTiming& beginFrameTiming(bool menuInputBlocking);

    void updateFrame(const RuntimeFrameInput& input);

    [[nodiscard]] const RuntimeFrameSnapshot& currentFrame();
    [[nodiscard]] bool isLocalSkeletonReady();
    [[nodiscard]] bool isPhysicsMenuBlocked();
    [[nodiscard]] bool isCompatibilityConfigBlocked();
}
