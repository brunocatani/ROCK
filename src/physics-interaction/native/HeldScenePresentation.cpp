#include "physics-interaction/native/HeldScenePresentation.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/HeldScenePresentationPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"

#include "RockConfig.h"

#include "RE/Havok/hknpBodyId.h"
#include "REL/Relocation.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
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

        constexpr std::array<std::uint8_t, 19> kExpectedWriterPrefix{
            0x4C, 0x8B, 0xDC,
            0x49, 0x89, 0x5B, 0x10,
            0x49, 0x89, 0x6B, 0x18,
            0x57,
            0x48, 0x81, 0xEC, 0x30, 0x01, 0x00, 0x00,
        };

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
            std::atomic<std::uint64_t> firstAppliedTraceId{ 0 };
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
        std::atomic<bool> s_installed{ false };
        SceneTransformWriter s_originalWriter = nullptr;

        AtomicHandRegistration& registrationFor(bool isLeft) noexcept
        {
            return s_registrations[isLeft ? 1u : 0u];
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
}
