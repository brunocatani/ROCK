// The per-frame runtime snapshot every ROCK subsystem reads: frame index and
// timing, menu and pause state, weapon drawn, skeleton readiness and the
// player-space frame. Built once per game frame (beginFrameTiming, updateFrame).
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
        bool wristPipboyOpen = false;
    };

    struct RuntimeFrameSnapshot
    {
        std::uint64_t frameIndex = 0;
        /*
         * Authoritative game-frame clock snapshot for this frame.
         * deltaSeconds is a convenience copy of its sanitized measured delta:
         * zero for an unmeasurable frame (every consumer holds on zero), the
         * clamped bounded delta for a hitch (timing.discontinuity tells
         * estimators to rebase).
         */
        game_frame_timing_policy::GameFrameTiming timing{};
        float deltaSeconds = 0.0f;
        bool playerAvailable = false;
        bool weaponDrawn = false;
        bool localMenuBlocking = false;
        bool localScopeMenuOpen = false;
        bool localLoadingMenuOpen = false;
        bool localGameStopped = false;
        /*
         * The engine's own pause: UI menu mode (UI+0x1E0) is nonzero. Unlike
         * localGameStopped (a fixed list of menu names), it tells the wrist
         * Pip-Boy, which keeps the game running, from the projected one.
         */
        bool localGamePaused = false;
        /*
         * The menu has closed and the native draw it left pending is still
         * running (an equip inside a paused menu plays its draw only after the
         * menu closes). The weapons stay presented as in the menu until the
         * draw ends, so the exit shows no gap and no hand step.
         */
        bool localMenuExitSettling = false;
        bool inputMenuBlocking = false;
        bool compatibilityConfigBlocking = false;
        // FRIK's wrist Pip-Boy, which opens a frame or more before PipboyMenu.
        bool wristPipboyOpen = false;
        bool visualAuthorityAvailable = false;
        bool visualSkeletonReadyHint = false;
        bool localSkeletonReady = false;
        bool localSkeletonRootAttached = false;
        bool localSkeletonRequiredHandBonesReady = false;
        PlayerSpaceFrame playerSpace{};

        // ROCK presents the equipped weapons itself this frame: they stay in
        // hand inside menus and through the draw a menu left pending.
        [[nodiscard]] bool menuHoldsWeapons() const noexcept { return localMenuBlocking || localMenuExitSettling; }
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
    // Pre-FRIK prediction runs before this frame's runtime snapshot exists.
    // Read the existing menu sink at that phase without advancing the snapshot.
    [[nodiscard]] bool isScopeMenuOpenNow();
    [[nodiscard]] bool isLocalSkeletonReady();
    [[nodiscard]] bool isPhysicsMenuBlocked();
    [[nodiscard]] bool isCompatibilityConfigBlocked();
    // Any thread: this frame's menuHoldsWeapons(), for native equip callbacks,
    // which run on the UI and job threads while a menu is open.
    [[nodiscard]] bool menuHoldsWeaponsAnyThread() noexcept;
}
