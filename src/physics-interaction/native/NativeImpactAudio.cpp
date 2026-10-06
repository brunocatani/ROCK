#include "physics-interaction/native/NativeImpactAudio.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/performance/ContactPairProfile.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/ImpactAudioPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"

#include <array>

namespace rock::native_impact_audio
{
    namespace
    {
        // FO4VR 1.2.72: 14061B923..14061B96B constructs this value and
        // 140D643D0 consumes its material IDs and severity.
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

        Listener originalListener = nullptr;
        Consumer originalConsumer = nullptr;
        bool active = false;

        struct ContactContext
        {
            std::uint32_t bodies[2]{};
            std::uint32_t layers[2]{};
            bool generated = false;
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

        std::uint32_t onConsumer(void* manager, const SoundEvent* event, void* source) noexcept
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactConsumer);
            if (active && currentContact && currentContact->generated &&
                impact_audio_policy::muteShellPair(currentContact->layers[0], currentContact->layers[1])) {
                // Mute only this sound event. The listener still executes
                // native collision/gameplay handling after the audio call.
                return 0;
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
            dispatch();
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
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !entryMatches(0x61B5C0, listenerEntry) || !entryMatches(0xD643D0, consumerEntry)) {
            ROCK_LOG_ERROR(Init, "Impact audio unavailable: native entry validation failed");
            return false;
        }
        void* original = nullptr;
        if (!entry_trampoline_hook::install("Impact audio consumer", 0xD643D0,
                consumerEntry.data(), consumerEntry.size(), reinterpret_cast<void*>(&onConsumer), original)) return false;
        originalConsumer = reinterpret_cast<Consumer>(original);
        if (!entry_trampoline_hook::install("Impact audio contact context", 0x61B5C0,
                listenerEntry.data(), listenerEntry.size(), reinterpret_cast<void*>(&onListener), original)) return false;
        originalListener = reinterpret_cast<Listener>(original);
        active = true;
        ROCK_LOG_INFO(Init, "Impact audio active: shell/generated audio muted; native physics preserved");
        return true;
    }

    void observeManifold(RE::hknpWorld* world, std::uint32_t bodyA,
        std::uint32_t bodyB, std::uint32_t shapeKeyA, std::uint32_t shapeKeyB) noexcept
    {
        if (!active || !performance_profiler::enabled()) return;
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativeImpactManifoldObserve);
        ContactContext context{};
        context.bodies[0] = bodyA;
        context.bodies[1] = bodyB;
        if (!readPair(world, context) || !context.generated) return;
        performance_profiler::observeContactPair({
            .world = reinterpret_cast<std::uintptr_t>(world),
            .bodyA = bodyA, .bodyB = bodyB, .shapeA = shapeKeyA, .shapeB = shapeKeyB,
            .layerA = context.layers[0], .layerB = context.layers[1],
        }, performance_profiler::ContactStage::Manifold);
    }
}
