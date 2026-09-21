#include "physics-interaction/native/NativeImpactAudio.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/performance/ContactPairProfile.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/ImpactAudioPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/telemetry/DynamicColliderTrace.h"

#include "RE/Havok/hknpBody.h"
#include <Windows.h>

#include <array>
#include <atomic>

namespace rock::native_impact_audio
{
    namespace
    {
        // FO4VR 1.2.72: 14061B923..14061B96B constructs this value;
        // 140D643D0 and 140D66240 consume its material IDs and severity.
        struct SoundEvent
        {
            std::uint32_t materialA;
            std::uint32_t materialB;
            float pointGame[3];
            float severity;
        };
        static_assert(sizeof(SoundEvent) == 0x18);

        using Listener = void (*)(void*, RE::hknpWorld**, const void*);
        using Consumer = std::uint32_t (*)(void*, const SoundEvent*, void*);
        using PlayPair = bool (*)(void*, void*, const SoundEvent*);
        using ShapeMaterial = std::uint32_t (*)(const RE::hknpShape*, std::uint32_t);

        Listener originalListener = nullptr;
        Consumer originalConsumer = nullptr;
        PlayPair originalPlayPair = nullptr;
        ShapeMaterial shapeMaterial = nullptr;
        bool active = false;

        struct ContactContext
        {
            std::uint32_t bodies[2]{};
            std::uint32_t layers[2]{};
            bool generated = false;
            bool dispatched = false;
            bool muted = false;
            std::uint32_t pairCalls = 0;
            std::uint32_t acceptedPairs = 0;
            SoundEvent sound{};
        };

        // The native listener calls the manager+0x10 consumer synchronously:
        // 140D91F7B installs vtable 142D78C70; its +8 slot is 140D643D0.
        // Stack-scoped TLS handles nested callbacks without sharing pointers
        // between threads or retaining a body/event beyond its native callback.
        thread_local ContactContext* currentContact = nullptr;
        struct ContactScope
        {
            ContactContext* previous = currentContact;
            explicit ContactScope(ContactContext& context) noexcept { currentContact = &context; }
            ~ContactScope() { currentContact = previous; }
        };

        struct RouteCounters
        {
            std::atomic<std::uint64_t> manifolds{ 0 };
            std::atomic<std::uint64_t> impulses{ 0 };
            std::atomic<std::uint64_t> nextManifoldMs{ 0 };
            std::atomic<std::uint64_t> nextImpulseMs{ 0 };
        };
        // Separate shell/world/other samples for weapon and hand/body contacts.
        std::array<RouteCounters, 6> routes;

        std::size_t routeFor(const ContactContext& context) noexcept
        {
            using namespace collision_layer_policy;
            const auto a = context.layers[0];
            const auto b = context.layers[1];
            const bool weapon = a == ROCK_LAYER_WEAPON || b == ROCK_LAYER_WEAPON ||
                a == ROCK_LAYER_DYNAMIC_WEAPON_PROXY || b == ROCK_LAYER_DYNAMIC_WEAPON_PROXY;
            const std::size_t peer = a == FO4_LAYER_SHELLCASING || b == FO4_LAYER_SHELLCASING ? 0 :
                (isWorldSurfaceLayer(a) || isWorldSurfaceLayer(b) ? 1 : 2);
            return (weapon ? 0 : 3) + peer;
        }

        bool sample(std::atomic<std::uint64_t>& next) noexcept
        {
            const auto now = GetTickCount64();
            auto expected = next.load(std::memory_order_relaxed);
            return now >= expected && next.compare_exchange_strong(expected, now + 250,
                std::memory_order_relaxed);
        }

        bool readPair(RE::hknpWorld* world, ContactContext& context) noexcept
        {
            std::uint32_t filterA = 0;
            std::uint32_t filterB = 0;
            if (!havok_runtime::tryReadFilterInfo(world, { context.bodies[0] }, filterA) ||
                !havok_runtime::tryReadFilterInfo(world, { context.bodies[1] }, filterB)) return false;
            context.layers[0] = filterA & collision_layer_policy::FO4_LAYER_FILTER_MASK;
            context.layers[1] = filterB & collision_layer_policy::FO4_LAYER_FILTER_MASK;
            context.generated = collision_layer_policy::isRockGeneratedColliderLayer(context.layers[0]) ||
                collision_layer_policy::isRockGeneratedColliderLayer(context.layers[1]);
            return true;
        }

        bool onPlayPair(void* material, void* otherMaterial, const SoundEvent* event) noexcept
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactPlayPair);
            const bool accepted = originalPlayPair(material, otherMaterial, event);
            if (currentContact && currentContact->generated) {
                ++currentContact->pairCalls;
                currentContact->acceptedPairs += accepted ? 1u : 0u;
            }
            return accepted;
        }

        std::uint32_t onConsumer(void* manager, const SoundEvent* event, void* source) noexcept
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactConsumer);
            if (active && currentContact && currentContact->generated) {
                currentContact->dispatched = true;
                if (event) currentContact->sound = *event;
                if (impact_audio_policy::muteShellPair(currentContact->layers[0], currentContact->layers[1])) {
                    currentContact->muted = true;
                    // Mute only this sound event. The listener still executes
                    // native collision/gameplay handling after the audio call.
                    return 0;
                }
            }
            return originalConsumer(manager, event, source);
        }

        void onListener(void* listener, RE::hknpWorld** worldHolder, const void* event) noexcept
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactListener);
            const auto dispatch = [&] {
                performance_profiler::ScopedTimer nativeTimer(performance_profiler::Scope::NativeImpactDispatch);
                originalListener(listener, worldHolder, event);
            };
            ContactContext context{};
            if (!active || !worldHolder || !*worldHolder ||
                !native_memory::tryReadField(event, 0x08, context.bodies) ||
                !readPair(*worldHolder, context) || !context.generated) {
                // Clear an outer generated context during nested native events.
                ContactScope scope(context);
                dispatch();
                return;
            }
            ContactScope scope(context);
            performance_profiler::observeContactPair({
                .world = reinterpret_cast<std::uintptr_t>(*worldHolder),
                .bodyA = context.bodies[0], .bodyB = context.bodies[1],
                .layerA = context.layers[0], .layerB = context.layers[1],
            }, performance_profiler::ContactStage::Impulse);
            auto& route = routes[routeFor(context)];
            const bool tracing = dynamic_collider_trace::enabled();
            const auto count = tracing ? route.impulses.fetch_add(1, std::memory_order_relaxed) + 1 : 0;
            const bool record = tracing && sample(route.nextImpulseMs);
            std::array<float, 4> impulses{};
            const bool weightsRead = record && native_memory::tryReadField(event, 0x30, impulses);
            dispatch();
            if (record) {
                dynamic_collider_trace::writeWeapon(
                    "IMPACT_AUDIO impulse: bodies={}/{} layers={}/{} impulses={} manifolds={} weightsRead={} weights=({:.4f},{:.4f},{:.4f},{:.4f}) dispatched={} shellMuted={} materials=0x{:08X}/0x{:08X} severity={:.4f} pairCalls={} acceptedPairs={}",
                    context.bodies[0], context.bodies[1], context.layers[0], context.layers[1], count,
                    route.manifolds.load(std::memory_order_relaxed), weightsRead,
                    impulses[0], impulses[1], impulses[2], impulses[3], context.dispatched, context.muted,
                    context.sound.materialA, context.sound.materialB, context.sound.severity,
                    context.pairCalls, context.acceptedPairs);
            }
        }

        template<std::size_t N>
        bool entryMatches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }
    }

    bool install() noexcept
    {
        if (active) return true;
        static bool attempted = false;
        if (attempted) return false;
        attempted = true;
        // Whole position-independent instructions, verified against raw VR
        // disassembly and the unpacked executable. No runtime tuning writes.
        constexpr std::array<std::uint8_t, 14> listenerEntry{
            0x48,0x8B,0xC4,0x48,0x89,0x50,0x10,0x48,0x89,0x48,0x08,0x55,0x53,0x56 };
        constexpr std::array<std::uint8_t, 14> consumerEntry{
            0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x18,0x56,0x57,0x41,0x56 };
        constexpr std::array<std::uint8_t, 16> pairEntry{
            0x40,0x53,0x48,0x83,0xEC,0x60,0x48,0x8B,0x49,0x58,0x49,0x8B,0xD8,0x48,0x85,0xC9 };
        constexpr std::array<std::uint8_t, 14> materialEntry{
            0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x74,0x24,0x20,0x89,0x54,0x24,0x10 };
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !entryMatches(0x61B5C0, listenerEntry) || !entryMatches(0xD643D0, consumerEntry) ||
            !entryMatches(0xD66240, pairEntry) || !entryMatches(0x1E14A60, materialEntry)) {
            ROCK_LOG_ERROR(Init, "Impact audio unavailable: native entry validation failed");
            return false;
        }
        void* original = nullptr;
        if (!entry_trampoline_hook::install("Impact audio consumer", 0xD643D0,
                consumerEntry.data(), consumerEntry.size(), reinterpret_cast<void*>(&onConsumer), original)) return false;
        originalConsumer = reinterpret_cast<Consumer>(original);
        if (!entry_trampoline_hook::install("Impact audio pair result", 0xD66240,
                pairEntry.data(), pairEntry.size(), reinterpret_cast<void*>(&onPlayPair), original)) return false;
        originalPlayPair = reinterpret_cast<PlayPair>(original);
        if (!entry_trampoline_hook::install("Impact audio contact context", 0x61B5C0,
                listenerEntry.data(), listenerEntry.size(), reinterpret_cast<void*>(&onListener), original)) return false;
        originalListener = reinterpret_cast<Listener>(original);
        shapeMaterial = reinterpret_cast<ShapeMaterial>(REL::Offset(0x1E14A60).address());
        active = true;
        ROCK_LOG_INFO(Init, "Impact audio active: shell/generated audio muted; native physics preserved; IMPACT_AUDIO diagnostics follow grab-frame trace in ROCK_WeaponContact.log");
        return true;
    }

    void observeManifold(RE::hknpWorld* world, std::uint32_t bodyA,
        std::uint32_t bodyB, std::uint32_t shapeKeyA, std::uint32_t shapeKeyB) noexcept
    {
        const bool tracing = dynamic_collider_trace::enabled();
        if (!active || (!tracing && !performance_profiler::enabled())) return;
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactManifoldTrace);
        ContactContext context{};
        context.bodies[0] = bodyA;
        context.bodies[1] = bodyB;
        if (!readPair(world, context) || !context.generated) return;
        performance_profiler::observeContactPair({
            .world = reinterpret_cast<std::uintptr_t>(world),
            .bodyA = bodyA, .bodyB = bodyB, .shapeA = shapeKeyA, .shapeB = shapeKeyB,
            .layerA = context.layers[0], .layerB = context.layers[1],
        }, performance_profiler::ContactStage::Manifold);
        if (!tracing) return;
        auto& route = routes[routeFor(context)];
        const auto count = route.manifolds.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!sample(route.nextManifoldMs)) return;
        const auto a = havok_runtime::snapshotBodyIdentity(world, { bodyA });
        const auto b = havok_runtime::snapshotBodyIdentity(world, { bodyB });
        if (!a.valid || !b.valid || !a.body->shape || !b.body->shape) return;
        const auto materialA = shapeMaterial(a.body->shape, shapeKeyA);
        const auto materialB = shapeMaterial(b.body->shape, shapeKeyB);
        dynamic_collider_trace::writeWeapon(
            "IMPACT_AUDIO manifold: bodies={}/{} layers={}/{} shapeKeys=0x{:08X}/0x{:08X} materials=0x{:08X}/0x{:08X} flags=0x{:08X}/0x{:08X} manifolds={} impulses={}",
            bodyA, bodyB, context.layers[0], context.layers[1], shapeKeyA, shapeKeyB,
            materialA, materialB, a.body->flags, b.body->flags, count,
            route.impulses.load(std::memory_order_relaxed));
    }
}
