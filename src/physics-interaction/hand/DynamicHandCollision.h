#pragma once

#include "physics-interaction/hand/DynamicHandTwinTargets.h"
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
     * overhaul-roadmap.md §8): DYNAMIC twins of the palm anchor and the five
     * fingertip colliders chase their published role frames with engine
     * hard-keyframe velocities every physics substep, on the world-only
     * extended layer. Static world clips their velocity inside the solver
     * (true multi-plane contact); the rendered FRIK hand follows the COMBINED
     * position deviation (sequential projection over per-body deviations, then
     * exponential smoothing against solver contact noise). Authority is
     * strictly one-directional (wand/skeleton targets -> twins -> render): the
     * twin targets come from the same role-frame publication the keyframed
     * colliders are driven with, never from the rendered hand, so rendering
     * cannot feed back into physics. The twins are not gameplay contact
     * evidence and collide only with static world-surface layers.
     *
     * Threading: updateFrame runs on the main game thread; the drive flush runs
     * on the physics step thread and publishes per-body deviations through
     * atomics that updateFrame consumes one substep later (~1/270 s).
     */
    class DynamicHandCollisionRuntime
    {
    public:
        static constexpr std::size_t kPalmSlot = 0;
        static constexpr std::size_t kBodiesPerHand = 1 + hand_collider_semantics::kHandFingerCount;

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
        [[nodiscard]] RE::hknpBodyId proxyBodyIdForDebug(bool isLeft, std::size_t bodyIndex) const
        {
            if (bodyIndex >= kBodiesPerHand) {
                return RE::hknpBodyId{ 0x7FFF'FFFF };
            }
            const auto& slot = _hands[isLeft ? 1u : 0u].bodies[bodyIndex];
            return slot.created ? slot.body.getBodyId() : RE::hknpBodyId{ 0x7FFF'FFFF };
        }

    private:
        struct ProxySlot
        {
            BethesdaPhysicsBody body{};
            RE::hknpShape* shape = nullptr;
            GeneratedKeyframedBodyDriveState driveState{};
            RE::hknpWorld* createdWorld = nullptr;
            void* createdBhkWorld = nullptr;
            float createdLength = 0.0f;
            float createdRadius = 0.0f;
            bool created = false;
            std::atomic<bool> deviationValidAtomic{ false };
            std::atomic<float> deviationXAtomic{ 0.0f };
            std::atomic<float> deviationYAtomic{ 0.0f };
            std::atomic<float> deviationZAtomic{ 0.0f };
            std::atomic<bool> rebuildRequestedAtomic{ false };
        };

        struct HandSlots
        {
            std::array<ProxySlot, kBodiesPerHand> bodies{};
            RE::NiPoint3 appliedDeviation{};
            bool visualActive = false;
        };

        bool ensureSlotCreated(ProxySlot& slot,
            bool isLeft,
            std::size_t bodyIndex,
            const PhysicsFrameContext& frame,
            const Hand& hand,
            const dynamic_hand_twin::TwinSlotFrame& twinFrame);
        void retireSlot(ProxySlot& slot, void* bhkWorld);
        void retireHand(HandSlots& handSlots, void* bhkWorld, bool isLeft);
        void clearVisual(HandSlots& handSlots, bool isLeft);

        std::array<HandSlots, 2> _hands{};
        std::uint32_t _logCounter = 0;
    };
}
