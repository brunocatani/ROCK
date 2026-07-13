#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpShape.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstdint>

namespace rock
{
    class Hand;
    struct PhysicsFrameContext;
    struct HandFrameInput;

    /*
     * Soft-collision overhaul stage A (Docs/ROCK/docs/2026-07-13-soft-collision-
     * overhaul-roadmap.md §8): one DYNAMIC sphere proxy per hand on the
     * world-only extended layer chases the raw tracked hand with engine
     * hard-keyframe velocities every physics substep. Static world clips the
     * proxy's velocity inside the solver (true multi-plane contact), and the
     * rendered FRIK hand follows the proxy's position deviation. Authority is
     * strictly one-directional (wand -> proxy -> render): the proxy target
     * never reads the rendered hand, so rendering cannot feed back into
     * physics. The proxy is not gameplay contact evidence and collides only
     * with static world-surface layers.
     *
     * Threading: updateFrame runs on the main game thread; the drive flush runs
     * on the physics step thread. The flush publishes the body-vs-target
     * deviation through per-slot atomics that updateFrame consumes one substep
     * later (the pre-collide body pose equals the previous substep's post-solve
     * pose, ~1/270 s of latency).
     */
    class DynamicHandCollisionRuntime
    {
    public:
        void updateFrame(const PhysicsFrameContext& frame,
            bool physicsWritesAllowed,
            const Hand& rightHand,
            const Hand& leftHand,
            bool rightHandWeaponEquipped,
            bool leftSupportGripActive);
        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void retireAll(void* bhkWorld);
        void reset();

        /*
         * Debug-overlay accessor; main thread only (creation/retire happen on
         * the same thread as the overlay publish).
         */
        [[nodiscard]] RE::hknpBodyId proxyBodyIdForDebug(bool isLeft) const
        {
            const auto& slot = _hands[isLeft ? 1u : 0u];
            return slot.created ? slot.proxyBody.getBodyId() : RE::hknpBodyId{ 0x7FFF'FFFF };
        }

    private:
        struct HandSlot
        {
            BethesdaPhysicsBody proxyBody{};
            RE::hknpShape* shape = nullptr;
            GeneratedKeyframedBodyDriveState driveState{};
            RE::hknpWorld* createdWorld = nullptr;
            void* createdBhkWorld = nullptr;
            bool created = false;
            bool visualActive = false;
            std::atomic<bool> deviationValidAtomic{ false };
            std::atomic<float> deviationXAtomic{ 0.0f };
            std::atomic<float> deviationYAtomic{ 0.0f };
            std::atomic<float> deviationZAtomic{ 0.0f };
            std::atomic<bool> rebuildRequestedAtomic{ false };
        };

        bool ensureCreated(HandSlot& slot, bool isLeft, const PhysicsFrameContext& frame, const HandFrameInput& handInput);
        void retireSlot(HandSlot& slot, void* bhkWorld, bool isLeft);
        void clearVisual(HandSlot& slot, bool isLeft);

        std::array<HandSlot, 2> _hands{};
        std::uint32_t _logCounter = 0;
    };
}
