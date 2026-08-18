#pragma once

#include <cstdint>

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiTransform.h"

/*
 * Held-body render-pose masquerade (2026-08-17 stick-locomotion buzz).
 *
 * The renderer consumes the held object's pose from the physics BODY path,
 * not from the NiAVObject writes ROCK makes (proven by the ±Z render probe:
 * offset node writes were invisible while the rendered object tracked the
 * debug colliders). The solver-driven body runs on the physics clock and
 * jitters against the render-clock camera, which is the visible buzz.
 *
 * This module makes the RENDERED body pose the producer's render-clock
 * anchor without the solver ever seeing it:
 *
 *   - producer (game thread) publishes the anchor pose as an hkTransform
 *     target while ROCK owns the render-clock node;
 *   - at the FINAL substep's post-solve callback (inside bhkWorld::Update,
 *     after the last solve, before the AfterWholePhysicsUpdate listeners
 *     and every post-update consumer) the held body's solver pose is saved
 *     and the anchor pose is written through the engine's own immediate
 *     hknpWorld::setBodyTransform (verified 0x1415395E0: preserves
 *     velocities, keeps motion/spatial/attached-body state coherent,
 *     requires world-write execution context and a valid live body id);
 *   - at the next update's BeforeWholePhysicsUpdate callback (before any
 *     collide) the saved solver pose is restored through the same routine,
 *     so collide/solve only ever consume solver state.
 *
 * On release the masquerade is dropped WITHOUT restoring: the body stays at
 * the last rendered pose, which is where the player saw the object (this
 * also removes the release snap-back). If the body was rewritten by someone
 * else inside the masquerade window (e.g. the engine keyframing a clutter
 * body on deactivation), the restore fails closed: it skips the write,
 * drops the masquerade, and counts the mismatch.
 *
 * Threading: publish/invalidate/clear run on the game thread; apply/restore
 * run in the physics step callbacks. Each hand slot is guarded by a mutex;
 * the physics side only try_locks and skips one step on contention (a lost
 * masquerade step renders one solver-pose frame, never blocks the step).
 */
namespace rock::held_body_render_pose
{
    // Diagnostic counters for the debug feed / rate-limited logging.
    struct MasqueradeStatus
    {
        std::uint32_t appliedSteps{ 0 };
        std::uint32_t restoredSteps{ 0 };
        std::uint32_t skippedContended{ 0 };
        std::uint32_t skippedInvalidBody{ 0 };
        std::uint32_t skippedImplausible{ 0 };
        std::uint32_t restoreMismatch{ 0 };
        std::uint32_t activeNow{ 0 };  // bitmask: 1 = right, 2 = left
    };

    // Game thread (producer). Publishes the render-clock anchor for the held
    // body; only frames where ROCK owns the rendered node may publish.
    // anchorWorldGame is the held node's anchor pose in game units.
    void publishTarget(bool isLeft, std::uint32_t bodyId, const RE::NiTransform& anchorWorldGame, float gameToHavok) noexcept;

    // Game thread. Invalidate the target so the next physics step applies no
    // masquerade (producer frames without node ownership, config disabled).
    void invalidateTarget(bool isLeft) noexcept;

    // Game thread. Release / world-loss: drop target and any active
    // masquerade WITHOUT restoring the solver pose (see header comment).
    void clearWithoutRestore(bool isLeft) noexcept;

    // Physics step callbacks (world-write execution context).
    void applyAtFinalSubstepPostSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept;
    void restoreBeforeWholeStep(RE::hknpWorld* world) noexcept;

    // Game thread. While the masquerade holds the body, ROCK's own
    // physics-truth readers must use the saved solver pose instead of the
    // body slot (which carries the anchor). Returns true and fills the
    // body-world transform in game units when an override applies.
    bool tryGetSolverPoseOverride(std::uint32_t bodyId, float havokToGame, RE::NiTransform& outBodyWorldGame) noexcept;

    // Copy of the diagnostic counters for the monitor feed.
    void copyStatus(MasqueradeStatus& out) noexcept;
}
