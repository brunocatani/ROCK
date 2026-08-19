#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include "physics-interaction/debug/overlay/DebugBodyOverlay.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

namespace rock
{
    using namespace physics_interaction_detail;

    // This file contains every physics-step writer except contact callbacks.
    // Runs on the physics step thread.
    void PhysicsInteraction::onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        /*
         * This pre-collide callback runs inside the same native Havok step path
         * used for generated body writes. It reasserts bit 14 only while the
         * cached native identity still owns the lease. This callback only reads
         * the cache; game-frame refresh owns stale-lease eviction under the
         * callback quiescence gate.
         */
        self->refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(world, "native-player-body-pre-collide");
        self->driveGeneratedCollidersFromPhysicsSubstep(world, timing);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::onCustomGrabAuthorityAfterCharacterMovement(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveCustomGrabAuthorityAfterCharacterMovement(world, timing);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::onCustomGrabAuthorityAfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->observeCustomGrabAuthorityAfterSolve(world, timing);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::onHeldBodyRenderPoseBeforeWholeStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample&)
    {
        /*
         * Restore must run before any collide even when a reset is in flight:
         * an unrestored masquerade would feed the anchor pose into the solve.
         * The module itself validates the body and fails closed, so only the
         * world pointer is gated here.
         */
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self || !world) {
            return;
        }
        held_body_render_pose::restoreBeforeWholeStep(world);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GeneratedColliderPhysicsFlush);

        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _bodyBoneColliders.flushPendingPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicHandCollision.flushPendingPhysicsDrive(world, timing);
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-collider-drive", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-collider-drive", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::driveCustomGrabAuthorityAfterCharacterMovement(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }
        if (!_rightHand.isHoldingAtomic() && !_leftHand.isHoldingAtomic()) {
            return;
        }

        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-character-move-before-grab-flush", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-character-move-before-grab-flush", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);

        // bhkWorld executes every listener's between-collide-and-solve graph
        // before invoking the +0x30 finish slot that owns this callback. The
        // character-manager movement task has therefore updated the live root,
        // while hknp solve has not started. One read is shared by both hands so
        // two-hand grabs receive an identical consumption-frame measurement.
        const auto controller = character_controller_runtime::samplePlayerCharacterControllerPositionHavok();
        const auto scale = physics_scale::current();
        const grab_authority_source_clock::ControllerRootFrameSample consumptionControllerRoot{
            .positionHavok = controller.positionHavok,
            .controllerIdentity = controller.controllerIdentity,
            .controllerVtable = controller.controllerVtable,
            .physicsScaleRevision = scale.revision,
            .valid = controller.valid,
        };
        _rightHand.flushPendingCustomGrabAuthority(world, timing, consumptionControllerRoot, scale.havokToGame);
        _leftHand.flushPendingCustomGrabAuthority(world, timing, consumptionControllerRoot, scale.havokToGame);
    }

    // Runs on the physics step thread.
    void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto completedSolveSequence =
            _completedPhysicsSolveSequence.fetch_add(
                1,
                std::memory_order_release) +
            1;
        _rightHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _leftHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _dynamicWeaponCollision.samplePostSolve(
            world,
            completedSolveSequence,
            timing);
        _dynamicHandCollision.samplePostSolveDeviations(
            world,
            completedSolveSequence,
            timing);
        /*
         * driveToKeyFrame programs velocity for the substep that follows the
         * pre-collide callback; it does not make that callback's body matrix a
         * solved pose. Capture only after the final solve, when every generated
         * body has actually consumed the current game-frame target. Publishing
         * from pre-collide exposed the previous solved transform as if it were
         * current and created an exact one-physics-step presentation delay.
         * Intermediate substeps are intentionally not admitted to Submit.
         */
        if (timing.substepCount > 0 &&
            timing.substepIndex + 1 >= timing.substepCount) {
            debug::CaptureAppliedGeneratedBodyTransformsFromPhysicsStep(world);
        }
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-solve", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-solve", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        serviceRetiredGrabConstraintPayloads();
        _weaponCollision.serviceRetiredWeaponBodies();
        // Frees hand/body bone-collider and grab-authority-proxy collision objects
        // that were world-removed on the main thread, only after the broadphase has
        // been rebuilt by this step. Runs here so all deferred collider teardown
        // shares the same post-solve grace cadence as weapon bodies and constraints.
        BethesdaPhysicsBody::serviceRetiredDeferredPayloads();

        // LAST post-solve action: every observer above must sample the true
        // solver pose before the held body is masqueraded to the render-clock
        // anchor for the remainder of the frame (module gates on the final
        // substep; the solver pose is restored in the before-whole callback).
        held_body_render_pose::applyAtFinalSubstepPostSolve(world, timing);
    }

}

