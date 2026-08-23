#include "physics-interaction/native/HeldScenePresentation.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/HeldScenePresentationPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/RendererOffsets.h"

#include "RockConfig.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "REL/Relocation.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <intrin.h>

namespace rock::held_scene_presentation
{
    namespace
    {
        using SceneTransformWriter = void (*)(void*, float*);
        using PredictBodyTransform = void (*)(
            RE::hknpWorld*,
            std::uint32_t,
            float,
            float*);
        using LightingShaderSetupGeometry = void (*)(void*, void*, void*);

        constexpr std::array<std::uint8_t, 19> kExpectedWriterPrefix{
            0x4C, 0x8B, 0xDC,
            0x49, 0x89, 0x5B, 0x10,
            0x49, 0x89, 0x6B, 0x18,
            0x57,
            0x48, 0x81, 0xEC, 0x30, 0x01, 0x00, 0x00,
        };

        constexpr std::array<std::uint8_t, 20> kExpectedLightingSetupPrefix{
            0x48, 0x8B, 0xC4,
            0x48, 0x89, 0x58, 0x08,
            0x48, 0x89, 0x68, 0x18,
            0x48, 0x89, 0x70, 0x20,
            0x48, 0x89, 0x50, 0x10,
            0x57,
        };

        constexpr std::size_t kRenderConsumptionCapacity = 128;
        constexpr std::uintptr_t kRenderPassGeometryOffset = 0x18;
        constexpr std::uintptr_t kShaderDescriptorTechniqueOffset = 0x40;
        constexpr std::uintptr_t kGeometryShaderPropertyOffset = 0x178;
        constexpr std::uintptr_t kShaderPropertyFlagsOffset = 0x30;
        static_assert(
            sizeof(RE::NiTransform) == 16 * sizeof(float),
            "Render-consumption samples require the native 16-float NiTransform layout");

        struct AtomicRegisteredBody
        {
            std::atomic<RE::NiCollisionObject*> collisionObject{ nullptr };
            std::atomic<RE::hknpWorld*> world{ nullptr };
            std::atomic<std::uint32_t> bodyId{ 0x7FFF'FFFFu };
        };

        /*
         * Grab/release publication and the native scene writer can execute on
         * different engine threads. Every component remains atomic to avoid a
         * C++ data race; the odd/even sequence rejects mixed registrations.
         */
        struct AtomicHandRegistration
        {
            std::atomic<std::uint64_t> sequence{ 0 };
            std::array<AtomicRegisteredBody, kMaxRegisteredBodies> bodies{};
            std::atomic<std::size_t> count{ 0 };
            std::atomic<std::uint64_t> traceId{ 0 };
            std::atomic<RE::NiAVObject*> visibleGeometry{ nullptr };
            std::atomic<std::uint64_t> firstAppliedTraceId{ 0 };
        };

        /*
         * The renderer hook cannot format or write logs. Each matched call
         * claims one fixed ring slot, publishes only atomic scalar fields, and
         * releases the completed ordinal. The game thread reads the newest
         * stable slot and owns all diagnostic logging.
         */
        struct AtomicRenderConsumption
        {
            std::atomic<std::uint64_t> readyOrdinal{ 0 };
            std::atomic<std::uint64_t> traceId{ 0 };
            std::atomic<std::uint64_t> captureMicroseconds{ 0 };
            std::atomic<std::uintptr_t> renderPass{ 0 };
            std::atomic<std::uint32_t> threadId{ 0 };
            std::atomic<std::uint32_t> technique{ 0 };
            std::atomic<RE::NiAVObject*> geometry{ nullptr };
            std::array<std::atomic<std::uint32_t>, 16> worldBits{};
            std::atomic<std::uint64_t> shaderFlagsBefore{ 0 };
            std::atomic<std::uint64_t> shaderFlagsAfter{ 0 };
        };

        struct Match
        {
            bool valid = false;
            bool isLeft = false;
            RE::hknpWorld* world = nullptr;
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint64_t traceId = 0;
        };

        std::array<AtomicHandRegistration, 2> s_registrations{};
        std::array<
            std::array<AtomicRenderConsumption, kRenderConsumptionCapacity>,
            2>
            s_renderConsumption{};
        std::array<std::atomic<std::uint64_t>, 2>
            s_renderConsumptionOrdinals{};
        std::atomic<bool> s_installed{ false };
        std::atomic<bool> s_renderProbeInstalled{ false };
        SceneTransformWriter s_originalWriter = nullptr;
        LightingShaderSetupGeometry s_originalLightingSetup = nullptr;

        AtomicHandRegistration& registrationFor(bool isLeft) noexcept
        {
            return s_registrations[isLeft ? 1u : 0u];
        }

        std::uint64_t readShaderPropertyFlags(
            const RE::NiAVObject* geometry) noexcept
        {
            if (!geometry) {
                return 0;
            }

            const auto* geometryBytes =
                reinterpret_cast<const std::byte*>(geometry);
            auto* shaderProperty =
                *reinterpret_cast<void* const*>(
                    geometryBytes + kGeometryShaderPropertyOffset);
            if (!shaderProperty) {
                return 0;
            }

            const auto* propertyBytes =
                reinterpret_cast<const std::byte*>(shaderProperty);
            return *reinterpret_cast<const std::uint64_t*>(
                propertyBytes + kShaderPropertyFlagsOffset);
        }

        bool matchVisibleGeometry(
            const AtomicHandRegistration& registration,
            RE::NiAVObject* geometry,
            std::uint64_t& traceId) noexcept
        {
            constexpr int kMaxAttempts = 2;
            for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
                const std::uint64_t begin =
                    registration.sequence.load(std::memory_order_acquire);
                if ((begin & 1u) != 0) {
                    continue;
                }

                auto* registeredGeometry =
                    registration.visibleGeometry.load(
                        std::memory_order_relaxed);
                const std::uint64_t registeredTraceId =
                    registration.traceId.load(std::memory_order_relaxed);
                const std::uint64_t end =
                    registration.sequence.load(std::memory_order_acquire);
                if (begin != end) {
                    continue;
                }
                if (registeredGeometry != geometry || !registeredGeometry) {
                    return false;
                }

                traceId = registeredTraceId;
                return true;
            }
            return false;
        }

        void publishRenderConsumption(
            std::size_t handIndex,
            std::uint64_t traceId,
            std::uint64_t captureMicroseconds,
            void* renderPass,
            std::uint32_t technique,
            RE::NiAVObject* geometry,
            const RE::NiTransform& world,
            std::uint64_t shaderFlagsBefore,
            std::uint64_t shaderFlagsAfter) noexcept
        {
            const std::uint64_t ordinal =
                s_renderConsumptionOrdinals[handIndex].fetch_add(
                    1,
                    std::memory_order_acq_rel) +
                1;
            auto& destination = s_renderConsumption[handIndex]
                [ordinal % kRenderConsumptionCapacity];
            destination.readyOrdinal.store(0, std::memory_order_release);
            destination.traceId.store(traceId, std::memory_order_relaxed);
            destination.captureMicroseconds.store(
                captureMicroseconds,
                std::memory_order_relaxed);
            destination.renderPass.store(
                reinterpret_cast<std::uintptr_t>(renderPass),
                std::memory_order_relaxed);
            destination.threadId.store(
                GetCurrentThreadId(),
                std::memory_order_relaxed);
            destination.technique.store(technique, std::memory_order_relaxed);
            destination.geometry.store(geometry, std::memory_order_relaxed);
            const auto* worldFloats = reinterpret_cast<const float*>(&world);
            for (std::size_t index = 0; index < 16; ++index) {
                destination.worldBits[index].store(
                    std::bit_cast<std::uint32_t>(worldFloats[index]),
                    std::memory_order_relaxed);
            }
            destination.shaderFlagsBefore.store(
                shaderFlagsBefore,
                std::memory_order_relaxed);
            destination.shaderFlagsAfter.store(
                shaderFlagsAfter,
                std::memory_order_relaxed);
            destination.readyOrdinal.store(ordinal, std::memory_order_release);
        }

        bool registrationContainsCollision(
            const AtomicHandRegistration& registration,
            RE::NiCollisionObject* collisionObject) noexcept
        {
            constexpr int kMaxAttempts = 4;
            for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
                const std::uint64_t begin =
                    registration.sequence.load(std::memory_order_acquire);
                if ((begin & 1u) != 0) {
                    continue;
                }

                const std::size_t count = (std::min)(
                    registration.count.load(std::memory_order_relaxed),
                    kMaxRegisteredBodies);
                bool found = false;
                for (std::size_t index = 0; index < count; ++index) {
                    if (registration.bodies[index].collisionObject.load(
                            std::memory_order_relaxed) == collisionObject) {
                        found = true;
                        break;
                    }
                }

                const std::uint64_t end =
                    registration.sequence.load(std::memory_order_acquire);
                if (begin == end) {
                    return found;
                }
            }
            return false;
        }

        bool anyRegistrationContainsCollision(
            RE::NiCollisionObject* collisionObject) noexcept
        {
            return registrationContainsCollision(
                       s_registrations[0], collisionObject) ||
                   registrationContainsCollision(
                       s_registrations[1], collisionObject);
        }

        bool findExactMatchInRegistration(
            const AtomicHandRegistration& registration,
            bool isLeft,
            RE::NiCollisionObject* collisionObject,
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            Match& outMatch) noexcept
        {
            constexpr int kMaxAttempts = 4;
            for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
                const std::uint64_t begin =
                    registration.sequence.load(std::memory_order_acquire);
                if ((begin & 1u) != 0) {
                    continue;
                }

                const std::size_t count = (std::min)(
                    registration.count.load(std::memory_order_relaxed),
                    kMaxRegisteredBodies);
                const std::uint64_t traceId =
                    registration.traceId.load(std::memory_order_relaxed);
                bool found = false;
                for (std::size_t index = 0; index < count; ++index) {
                    const auto& entry = registration.bodies[index];
                    if (entry.collisionObject.load(std::memory_order_relaxed) == collisionObject &&
                        entry.world.load(std::memory_order_relaxed) == world &&
                        entry.bodyId.load(std::memory_order_relaxed) == bodyId) {
                        found = true;
                        break;
                    }
                }

                const std::uint64_t end =
                    registration.sequence.load(std::memory_order_acquire);
                if (begin != end) {
                    continue;
                }
                if (!found) {
                    return false;
                }

                outMatch = Match{
                    .valid = true,
                    .isLeft = isLeft,
                    .world = world,
                    .bodyId = bodyId,
                    .traceId = traceId,
                };
                return true;
            }
            return false;
        }

        bool findExactMatch(
            RE::NiCollisionObject* collisionObject,
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            Match& outMatch) noexcept
        {
            return findExactMatchInRegistration(
                       s_registrations[0],
                       false,
                       collisionObject,
                       world,
                       bodyId,
                       outMatch) ||
                   findExactMatchInRegistration(
                       s_registrations[1],
                       true,
                       collisionObject,
                       world,
                       bodyId,
                       outMatch);
        }

        void logFirstApplication(
            const Match& match,
            const held_scene_presentation_policy::TimingDecision& timing,
            const held_scene_presentation_policy::TransformDecision& transform,
            const float* originalInput,
            const float* correctedInput) noexcept
        {
            auto& registration = registrationFor(match.isLeft);
            std::uint64_t previous =
                registration.firstAppliedTraceId.load(std::memory_order_acquire);
            if (previous == match.traceId ||
                !registration.firstAppliedTraceId.compare_exchange_strong(
                    previous,
                    match.traceId,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                return;
            }

            ROCK_LOG_INFO(HeldScenePresentation,
                "HELD_SCENE_PRESENT first-apply trace={} hand={} body={} raw={:.6f}s remainder={:.6f}s prediction={:.6f}s delta={:.4f}gu rotation={:.3f}deg input=({:.3f},{:.3f},{:.3f}) output=({:.3f},{:.3f},{:.3f}) thread={}",
                match.traceId,
                match.isLeft ? "left" : "right",
                match.bodyId,
                timing.rawFrameSeconds,
                timing.nativeRemainderSeconds,
                timing.predictionSeconds,
                transform.translationDeltaGameUnits,
                transform.rotationDeltaDegrees,
                originalInput[12],
                originalInput[13],
                originalInput[14],
                correctedInput[12],
                correctedInput[13],
                correctedInput[14],
                GetCurrentThreadId());
        }

        void logRejectedDecision(
            const Match& match,
            const held_scene_presentation_policy::TimingDecision& timing,
            held_scene_presentation_policy::RejectReason reason) noexcept
        {
            if (!g_rockConfig.rockDebugGrabFrameLogging &&
                !g_rockConfig.rockDebugVerboseLogging) {
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(HeldScenePresentation,
                1000,
                "HELD_SCENE_PRESENT rejected trace={} hand={} body={} reason={} raw={:.6f}s remainder={:.9f}s prediction={:.6f}s",
                match.traceId,
                match.isLeft ? "left" : "right",
                match.bodyId,
                held_scene_presentation_policy::rejectReasonName(reason),
                timing.rawFrameSeconds,
                timing.nativeRemainderSeconds,
                timing.predictionSeconds);
        }

        void logSampledApplication(
            const Match& match,
            const held_scene_presentation_policy::TimingDecision& timing,
            const held_scene_presentation_policy::TransformDecision& transform)
            noexcept
        {
            if (!g_rockConfig.rockDebugGrabFrameLogging &&
                !g_rockConfig.rockDebugVerboseLogging) {
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(HeldScenePresentation,
                1000,
                "HELD_SCENE_PRESENT applied trace={} hand={} body={} raw={:.6f}s remainder={:.9f}s prediction={:.6f}s delta={:.4f}gu rotation={:.3f}deg",
                match.traceId,
                match.isLeft ? "left" : "right",
                match.bodyId,
                timing.rawFrameSeconds,
                timing.nativeRemainderSeconds,
                timing.predictionSeconds,
                transform.translationDeltaGameUnits,
                transform.rotationDeltaDegrees);
        }

        __declspec(noinline) void sceneTransformWriterHook(
            void* collisionObjectRaw,
            float* writerInput) noexcept
        {
            if (!s_originalWriter) {
                return;
            }

            auto* collisionObject =
                static_cast<RE::NiCollisionObject*>(collisionObjectRaw);
            if (!collisionObject || !writerInput ||
                !anyRegistrationContainsCollision(collisionObject)) {
                s_originalWriter(collisionObjectRaw, writerInput);
                return;
            }

            RE::hknpWorld* world = nullptr;
            RE::hknpBodyId bodyId{ 0x7FFF'FFFFu };
            Match match{};
            if (!havok_runtime::tryResolveCollisionObjectBody(
                    collisionObject,
                    world,
                    bodyId) ||
                !findExactMatch(
                    collisionObject,
                    world,
                    bodyId.value,
                    match)) {
                s_originalWriter(collisionObjectRaw, writerInput);
                return;
            }

            const auto returnAddress =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            static const std::uintptr_t mainWriterReturnAddress =
                REL::Offset(
                    offsets::kReturn_SceneTransformWriterMain).address();
            const bool mainCallsite =
                returnAddress == mainWriterReturnAddress;

            static REL::Relocation<float*> rawFrameSeconds{
                REL::Offset(offsets::kData_BhkWorldRawDeltaSeconds)
            };
            static REL::Relocation<float*> nativeRemainderSeconds{
                REL::Offset(offsets::kData_BhkWorldRemainderDeltaSeconds)
            };
            const auto timing =
                held_scene_presentation_policy::evaluateTiming(
                    mainCallsite,
                    *rawFrameSeconds,
                    *nativeRemainderSeconds);
            if (!timing.apply) {
                logRejectedDecision(match, timing, timing.reason);
                s_originalWriter(collisionObjectRaw, writerInput);
                return;
            }

            alignas(16) float nativePrediction[
                held_scene_presentation_policy::kPredictionFloatCount]{};
            static REL::Relocation<PredictBodyTransform> predictBodyTransform{
                REL::Offset(offsets::kFunc_PredictBodyTransform)
            };
            predictBodyTransform(
                match.world,
                match.bodyId,
                timing.predictionSeconds,
                nativePrediction);

            alignas(16) float correctedInput[
                held_scene_presentation_policy::kPredictionFloatCount]{};
            const auto transform =
                held_scene_presentation_policy::buildWriterTransform(
                    writerInput,
                    nativePrediction,
                    physics_scale::havokToGame(),
                    correctedInput);
            if (!transform.apply) {
                logRejectedDecision(match, timing, transform.reason);
                s_originalWriter(collisionObjectRaw, writerInput);
                return;
            }

            logFirstApplication(
                match,
                timing,
                transform,
                writerInput,
                correctedInput);
            logSampledApplication(match, timing, transform);
            s_originalWriter(collisionObjectRaw, correctedInput);
        }

        __declspec(noinline) void lightingShaderSetupHook(
            void* shader,
            void* renderPass,
            void* shaderDescriptor) noexcept
        {
            if (!s_originalLightingSetup) {
                return;
            }

            RE::NiAVObject* geometry = nullptr;
            if (renderPass) {
                const auto* passBytes =
                    reinterpret_cast<const std::byte*>(renderPass);
                geometry = *reinterpret_cast<RE::NiAVObject* const*>(
                    passBytes + kRenderPassGeometryOffset);
            }

            std::array<bool, 2> matchedHands{};
            std::array<std::uint64_t, 2> traceIds{};
            bool anyMatch = false;
            if (geometry) {
                for (std::size_t handIndex = 0;
                     handIndex < s_registrations.size();
                     ++handIndex) {
                    matchedHands[handIndex] = matchVisibleGeometry(
                        s_registrations[handIndex],
                        geometry,
                        traceIds[handIndex]);
                    anyMatch = anyMatch || matchedHands[handIndex];
                }
            }

            if (!anyMatch) {
                s_originalLightingSetup(shader, renderPass, shaderDescriptor);
                return;
            }

            const auto captureMicroseconds = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            const RE::NiTransform consumedWorld = geometry->world;
            const std::uint64_t shaderFlagsBefore =
                readShaderPropertyFlags(geometry);
            std::uint32_t technique = 0;
            if (shaderDescriptor) {
                const auto* descriptorBytes =
                    reinterpret_cast<const std::byte*>(shaderDescriptor);
                technique = *reinterpret_cast<const std::uint32_t*>(
                    descriptorBytes + kShaderDescriptorTechniqueOffset);
            }

            s_originalLightingSetup(shader, renderPass, shaderDescriptor);

            const std::uint64_t shaderFlagsAfter =
                readShaderPropertyFlags(geometry);
            for (std::size_t handIndex = 0;
                 handIndex < matchedHands.size();
                 ++handIndex) {
                if (!matchedHands[handIndex]) {
                    continue;
                }
                publishRenderConsumption(
                    handIndex,
                    traceIds[handIndex],
                    captureMicroseconds,
                    renderPass,
                    technique,
                    geometry,
                    consumedWorld,
                    shaderFlagsBefore,
                    shaderFlagsAfter);
            }
        }
    }

    bool install() noexcept
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(Init,
                "Held scene presentation unavailable: unsupported runtime");
            return false;
        }

        void* original = reinterpret_cast<void*>(s_originalWriter);
        const bool installed = entry_trampoline_hook::install(
            "held scene transform writer",
            offsets::kFunc_SceneTransformWriter,
            kExpectedWriterPrefix.data(),
            kExpectedWriterPrefix.size(),
            reinterpret_cast<void*>(&sceneTransformWriterHook),
            original);
        s_originalWriter = reinterpret_cast<SceneTransformWriter>(original);
        const bool ready = installed && s_originalWriter != nullptr;
        s_installed.store(ready, std::memory_order_release);
        return ready;
    }

    bool installRenderConsumptionProbe() noexcept
    {
        if (s_renderProbeInstalled.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(Init,
                "Held render-consumption probe unavailable: unsupported runtime");
            return false;
        }

        void* original = reinterpret_cast<void*>(s_originalLightingSetup);
        const bool installed = entry_trampoline_hook::install(
            "held lighting-shader geometry setup",
            renderer_offsets::kFunc_BSLightingShaderSetupGeometry,
            kExpectedLightingSetupPrefix.data(),
            kExpectedLightingSetupPrefix.size(),
            reinterpret_cast<void*>(&lightingShaderSetupHook),
            original);
        s_originalLightingSetup =
            reinterpret_cast<LightingShaderSetupGeometry>(original);
        const bool ready = installed && s_originalLightingSetup != nullptr;
        s_renderProbeInstalled.store(ready, std::memory_order_release);
        return ready;
    }

    void publishHeldBodies(
        bool isLeft,
        const Registration& registration) noexcept
    {
        auto& destination = registrationFor(isLeft);
        destination.sequence.fetch_add(
            1,
            std::memory_order_acq_rel);  // odd: write in progress

        const std::size_t count =
            (std::min)(registration.count, kMaxRegisteredBodies);
        destination.count.store(0, std::memory_order_relaxed);
        for (std::size_t index = 0; index < kMaxRegisteredBodies; ++index) {
            const RegisteredBody entry =
                index < count ? registration.bodies[index] : RegisteredBody{};
            destination.bodies[index].collisionObject.store(
                entry.collisionObject,
                std::memory_order_relaxed);
            destination.bodies[index].world.store(
                entry.world,
                std::memory_order_relaxed);
            destination.bodies[index].bodyId.store(
                entry.bodyId,
                std::memory_order_relaxed);
        }
        destination.traceId.store(
            registration.traceId,
            std::memory_order_relaxed);
        destination.visibleGeometry.store(
            registration.visibleGeometry,
            std::memory_order_relaxed);
        destination.count.store(count, std::memory_order_relaxed);
        destination.sequence.fetch_add(
            1,
            std::memory_order_release);  // even: complete registration
    }

    void clearHeldBodies(bool isLeft) noexcept
    {
        Registration empty{};
        publishHeldBodies(isLeft, empty);
    }

    bool readLatestRenderConsumption(
        bool isLeft,
        std::uint64_t afterOrdinal,
        RenderConsumptionSample& sample) noexcept
    {
        sample = {};
        const std::size_t handIndex = isLeft ? 1u : 0u;
        constexpr int kMaxAttempts = 3;
        for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
            const std::uint64_t latestOrdinal =
                s_renderConsumptionOrdinals[handIndex].load(
                    std::memory_order_acquire);
            if (latestOrdinal <= afterOrdinal) {
                return false;
            }

            const auto& source = s_renderConsumption[handIndex]
                [latestOrdinal % kRenderConsumptionCapacity];
            const std::uint64_t readyBefore =
                source.readyOrdinal.load(std::memory_order_acquire);
            if (readyBefore != latestOrdinal) {
                continue;
            }

            RenderConsumptionSample snapshot{};
            snapshot.ordinal = latestOrdinal;
            snapshot.traceId =
                source.traceId.load(std::memory_order_relaxed);
            snapshot.captureMicroseconds =
                source.captureMicroseconds.load(std::memory_order_relaxed);
            snapshot.renderPass =
                source.renderPass.load(std::memory_order_relaxed);
            snapshot.threadId =
                source.threadId.load(std::memory_order_relaxed);
            snapshot.technique =
                source.technique.load(std::memory_order_relaxed);
            snapshot.geometry =
                source.geometry.load(std::memory_order_relaxed);
            auto* worldFloats = reinterpret_cast<float*>(&snapshot.world);
            for (std::size_t index = 0; index < 16; ++index) {
                worldFloats[index] = std::bit_cast<float>(
                    source.worldBits[index].load(
                        std::memory_order_relaxed));
            }
            snapshot.shaderFlagsBefore =
                source.shaderFlagsBefore.load(std::memory_order_relaxed);
            snapshot.shaderFlagsAfter =
                source.shaderFlagsAfter.load(std::memory_order_relaxed);

            const std::uint64_t readyAfter =
                source.readyOrdinal.load(std::memory_order_acquire);
            if (readyBefore == readyAfter) {
                sample = snapshot;
                return true;
            }
        }
        return false;
    }
}
