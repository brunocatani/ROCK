#pragma once

#include "physics-interaction/actor/NpcCollisionSource.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "RE/Bethesda/BSPointerHandle.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE { class NiAVObject; }
namespace rock::debug { struct BodyOverlayFrame; }

namespace rock
{
    class PhysicsCallbackQuiescenceGate;
    struct PhysicsFrameContext;

    // Owns supplemental bodies only. NPC selection and source lifetime stay
    // here so this module can move to SCISSORS without changing ROCK providers.
    // update/clear run on the game thread under the existing callback gate;
    // drive runs on its pre-collision callback. No actor/node pointer is retained.
    class NpcCollisionRuntime
    {
    public:
        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate) { _gate = gate; }
        void update(const PhysicsFrameContext& frame, bool enabled, bool visualize = false);
        void drive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);
        void clear(RE::hknpWorld* liveWorld, void* liveBhkWorld);
        // Synchronous game-thread diagnostic publication; never changes native bodies.
        void appendDebugOverlay(debug::BodyOverlayFrame& frame, bool native, bool supplemental, bool labels);

    private:
        static constexpr std::size_t kActorCapacity = npc_collision::kActorCapacity;
        static constexpr std::size_t kBodyCapacity = npc_collision::kBodyCapacity;

        struct Limb
        {
            npc_collision::SourceIdentity source{};
            BethesdaPhysicsBody body{};
            bool collisionEnabled = false;
            bool failed = false;
            bool seen = false;
            std::array<char, 40> boneName{};
        };

        struct ActorSlot
        {
            RE::ActorHandle handle{};
            std::uint32_t formId = 0;
            std::uintptr_t root = 0;
            std::array<Limb, kBodyCapacity> limbs{};
        };

        void retire(Limb& limb, bool worldLive);
        void release(ActorSlot& actor, bool worldLive);
        void discover(const PhysicsFrameContext& frame);
        void scan(ActorSlot& slot, RE::NiAVObject* root);
        void disable(Limb& limb);

        PhysicsCallbackQuiescenceGate* _gate = nullptr;
        RE::hknpWorld* _world = nullptr;
        void* _bhkWorld = nullptr;
        std::array<ActorSlot, kActorCapacity> _actors{};
        float _discoverySeconds = 0.0f;
        float _mainFrameAgeSeconds = 0.0f;
        bool _companionChecked = false;
        bool _scissorsLoaded = false;
        std::uint32_t _driveFailures = 0;
        std::uint32_t _staleFrames = 0;
        std::size_t _lastActiveCount = 0;
        std::size_t _lastPooledCount = 0;
    };
}
