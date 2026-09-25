#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/telemetry/HeldRenderTrace.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "rock_support/Fo4VrRuntime.h"
#include "physics-interaction/grab/HeldScenePresentationPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"

#include "RockConfig.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "REL/Relocation.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <intrin.h>
#include <mutex>

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

        constexpr float kTargetWriterMaxTranslationDeltaGameUnits = 50.0f;
        constexpr float kTargetWriterMaxRotationDeltaDegrees = 75.0f;
        static_assert(
            sizeof(RE::NiTransform) == 16 * sizeof(float),
            "Target presentation requires the native 16-float NiTransform layout");

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
            std::atomic<bool> complete{ true };
        };

        struct TargetTransportHistory
        {
            RE::hknpWorld* world = nullptr;
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint64_t traceId = 0;
            RE::NiTransform previousTargetWorld{};
            bool valid = false;
        };

        struct AtomicBodyPose
        {
            std::atomic<std::uint32_t> bodyId{ 0x7FFF'FFFFu };
            std::array<std::atomic<std::uint32_t>, 16> worldBits{};
            std::atomic<std::uint64_t> lastLoggedFrame{ 0 };
            std::atomic<std::uint64_t> firstAppliedTraceId{ 0 };
        };

        struct BodyPose
        {
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            RE::NiTransform world{};
        };

        struct AtomicTargetTransport
        {
            std::atomic<std::uint64_t> sequence{ 0 };
            std::atomic<RE::hknpWorld*> world{ nullptr };
            std::atomic<std::uint64_t> traceId{ 0 };
            std::array<AtomicBodyPose, kMaxRegisteredBodies> bodies{};
            std::atomic<std::size_t> count{ 0 };
            std::atomic<std::uint64_t> frameIndex{ 0 };
            std::atomic<std::uint32_t> targetTranslationStepBits{ 0 };
            std::atomic<std::uint32_t> targetRotationStepBits{ 0 };
            std::atomic<std::uint32_t> physicalResidualBits{ 0 };
            std::atomic<std::uint32_t> transportAdvanceBits{ 0 };
            std::atomic<bool> valid{ false };
        };

        struct TargetTransportMatch
        {
            bool valid = false;
            bool isLeft = false;
            std::uint64_t traceId = 0;
            RE::NiTransform presentedWorld{};
            std::uint64_t frameIndex = 0;
            std::size_t bodyIndex = 0;
            float targetTranslationStepGameUnits = 0.0f;
            float targetRotationStepDegrees = 0.0f;
            float physicalResidualGameUnits = 0.0f;
            float transportAdvanceGameUnits = 0.0f;
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
        std::array<AtomicTargetTransport, 2> s_targetTransport{};
        std::array<TargetTransportHistory, 2> s_targetTransportHistory{};
        std::mutex s_targetTransportHistoryMutex;
        std::array<std::uint64_t, 2> s_lastAssemblyLogFrame{};
        std::array<std::uint64_t, 2> s_lastAssemblyLogTrace{};
        std::atomic<bool> s_installed{ false };
        SceneTransformWriter s_originalWriter = nullptr;

        AtomicHandRegistration& registrationFor(bool isLeft) noexcept
        {
            return s_registrations[isLeft ? 1u : 0u];
        }

        void clearTargetTransportPublication(std::size_t handIndex) noexcept
        {
            auto& destination = s_targetTransport[handIndex];
            destination.sequence.fetch_add(1, std::memory_order_acq_rel);
            destination.valid.store(false, std::memory_order_relaxed);
            destination.world.store(nullptr, std::memory_order_relaxed);
            destination.count.store(0, std::memory_order_relaxed);
            destination.traceId.store(0, std::memory_order_relaxed);
            destination.sequence.fetch_add(1, std::memory_order_release);
        }

        void resetTargetTransport(std::size_t handIndex) noexcept
        {
            std::scoped_lock lock(s_targetTransportHistoryMutex);
            s_targetTransportHistory[handIndex] = {};
            clearTargetTransportPublication(handIndex);
        }

        void publishTargetTransportDecision(
            std::size_t handIndex,
            RE::hknpWorld* world,
            const BodyPose* poses,
            std::size_t count,
            std::uint64_t traceId,
            std::uint64_t frameIndex,
            const held_scene_presentation_policy::TargetTransportDecision<
                RE::NiTransform>& decision) noexcept
        {
            auto& destination = s_targetTransport[handIndex];
            destination.sequence.fetch_add(1, std::memory_order_acq_rel);
            destination.valid.store(false, std::memory_order_relaxed);
            destination.world.store(world, std::memory_order_relaxed);
            destination.count.store(count, std::memory_order_relaxed);
            destination.frameIndex.store(frameIndex, std::memory_order_relaxed);
            destination.traceId.store(traceId, std::memory_order_relaxed);
            for (std::size_t bodyIndex = 0; bodyIndex < count; ++bodyIndex) {
                auto& body = destination.bodies[bodyIndex];
                body.bodyId.store(poses[bodyIndex].bodyId, std::memory_order_relaxed);
                const auto* worldFloats = reinterpret_cast<const float*>(&poses[bodyIndex].world);
                for (std::size_t index = 0; index < 16; ++index) {
                    body.worldBits[index].store(
                        std::bit_cast<std::uint32_t>(worldFloats[index]), std::memory_order_relaxed);
                }
            }
            destination.targetTranslationStepBits.store(
                std::bit_cast<std::uint32_t>(
                    decision.targetTranslationStepGameUnits),
                std::memory_order_relaxed);
            destination.targetRotationStepBits.store(
                std::bit_cast<std::uint32_t>(
                    decision.targetRotationStepDegrees),
                std::memory_order_relaxed);
            destination.physicalResidualBits.store(
                std::bit_cast<std::uint32_t>(
                    decision.physicalResidualGameUnits),
                std::memory_order_relaxed);
            destination.transportAdvanceBits.store(
                std::bit_cast<std::uint32_t>(
                    decision.transportAdvanceGameUnits),
                std::memory_order_relaxed);
            destination.valid.store(true, std::memory_order_relaxed);
            destination.sequence.fetch_add(1, std::memory_order_release);
        }

        bool copyTargetTransport(
            const AtomicTargetTransport& source,
            bool isLeft,
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            TargetTransportMatch& match) noexcept
        {
            constexpr int kMaxAttempts = 4;
            for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
                const std::uint64_t begin =
                    source.sequence.load(std::memory_order_acquire);
                if ((begin & 1u) != 0) {
                    continue;
                }
                if (!source.valid.load(std::memory_order_relaxed) ||
                    source.world.load(std::memory_order_relaxed) != world) {
                    return false;
                }

                const auto count = (std::min)(source.count.load(std::memory_order_relaxed), kMaxRegisteredBodies);
                std::size_t bodyIndex = 0;
                for (; bodyIndex < count; ++bodyIndex) {
                    if (source.bodies[bodyIndex].bodyId.load(std::memory_order_relaxed) == bodyId) {
                        break;
                    }
                }
                if (bodyIndex == count) {
                    if (begin != source.sequence.load(std::memory_order_acquire)) {
                        continue;
                    }
                    return false;
                }
                TargetTransportMatch snapshot{};
                snapshot.valid = true;
                snapshot.isLeft = isLeft;
                snapshot.frameIndex = source.frameIndex.load(std::memory_order_relaxed);
                snapshot.bodyIndex = bodyIndex;
                snapshot.traceId =
                    source.traceId.load(std::memory_order_relaxed);
                auto* worldFloats = reinterpret_cast<float*>(
                    &snapshot.presentedWorld);
                for (std::size_t index = 0; index < 16; ++index) {
                    worldFloats[index] = std::bit_cast<float>(
                        source.bodies[bodyIndex].worldBits[index].load(
                            std::memory_order_relaxed));
                }
                snapshot.targetTranslationStepGameUnits =
                    std::bit_cast<float>(
                        source.targetTranslationStepBits.load(
                            std::memory_order_relaxed));
                snapshot.targetRotationStepDegrees = std::bit_cast<float>(
                    source.targetRotationStepBits.load(
                        std::memory_order_relaxed));
                snapshot.physicalResidualGameUnits = std::bit_cast<float>(
                    source.physicalResidualBits.load(
                        std::memory_order_relaxed));
                snapshot.transportAdvanceGameUnits = std::bit_cast<float>(
                    source.transportAdvanceBits.load(
                        std::memory_order_relaxed));

                const std::uint64_t end =
                    source.sequence.load(std::memory_order_acquire);
                if (begin == end) {
                    match = snapshot;
                    return true;
                }
            }
            return false;
        }

        bool copyRegistration(bool isLeft, Registration& result) noexcept
        {
            const auto& source = registrationFor(isLeft);
            for (int attempt = 0; attempt < 4; ++attempt) {
                const auto begin = source.sequence.load(std::memory_order_acquire);
                if (begin & 1u) {
                    continue;
                }
                result.count = (std::min)(source.count.load(std::memory_order_relaxed), kMaxRegisteredBodies);
                result.traceId = source.traceId.load(std::memory_order_relaxed);
                result.complete = source.complete.load(std::memory_order_relaxed);
                for (std::size_t index = 0; index < result.count; ++index) {
                    const auto& body = source.bodies[index];
                    result.bodies[index] = {
                        body.collisionObject.load(std::memory_order_relaxed),
                        body.world.load(std::memory_order_relaxed),
                        body.bodyId.load(std::memory_order_relaxed),
                    };
                }
                if (begin == source.sequence.load(std::memory_order_acquire)) {
                    return true;
                }
            }
            return false;
        }

        bool containsBody(const Registration& registration, RE::hknpWorld* world, std::uint32_t bodyId) noexcept
        {
            for (std::size_t index = 0; index < registration.count; ++index) {
                const auto& body = registration.bodies[index];
                if (body.world == world && body.bodyId == bodyId) {
                    return true;
                }
            }
            return false;
        }

        bool findTargetTransport(
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            TargetTransportMatch& match) noexcept
        {
            Registration right{}, left{};
            if (!copyRegistration(false, right) || !copyRegistration(true, left)) {
                return false;
            }
            const bool rightOwns = containsBody(right, world, bodyId);
            const bool leftOwns = containsBody(left, world, bodyId);
            if (!rightOwns && !leftOwns) {
                return false;
            }
            // Registration, not publication readiness, owns arbitration. A
            // warming or rejected older hold must not switch individual parts
            // onto the other hand's clock.
            const bool isLeft = leftOwns && (!rightOwns ||
                held_scene_presentation_policy::preferEarlierTrace(left.traceId, right.traceId));
            return copyTargetTransport(s_targetTransport[isLeft ? 1u : 0u], isLeft, world, bodyId, match) &&
                   match.traceId == (isLeft ? left.traceId : right.traceId);
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

        void logTargetApplication(
            const TargetTransportMatch& target,
            std::uint32_t bodyId,
            const held_scene_presentation_policy::TransformDecision& transform,
            const float* correctedInput) noexcept
        {
            auto& body = s_targetTransport[target.isLeft ? 1u : 0u].bodies[target.bodyIndex];
            std::uint64_t previous =
                body.firstAppliedTraceId.load(
                    std::memory_order_acquire);
            if (previous != target.traceId &&
                body.firstAppliedTraceId.compare_exchange_strong(
                    previous,
                    target.traceId,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                ROCK_LOG_INFO(HeldScenePresentation,
                    "HELD_SCENE_TARGET first-apply trace={} frame={} hand={} body={} targetStep={:.4f}gu/{:.3f}deg residual={:.4f}gu advance={:.4f}gu writerDelta={:.4f}gu/{:.3f}deg output=({:.3f},{:.3f},{:.3f}) thread={}",
                    target.traceId,
                    target.frameIndex,
                    target.isLeft ? "left" : "right",
                    bodyId,
                    target.targetTranslationStepGameUnits,
                    target.targetRotationStepDegrees,
                    target.physicalResidualGameUnits,
                    target.transportAdvanceGameUnits,
                    transform.translationDeltaGameUnits,
                    transform.rotationDeltaDegrees,
                    correctedInput[12],
                    correctedInput[13],
                    correctedInput[14],
                    GetCurrentThreadId());
                body.lastLoggedFrame.store(target.frameIndex, std::memory_order_relaxed);
                return;
            }

            if (!g_rockConfig.rockDebugGrabFrameLogging &&
                !g_rockConfig.rockDebugVerboseLogging) {
                return;
            }
            auto& lastFrame = s_targetTransport[target.isLeft ? 1u : 0u].bodies[target.bodyIndex].lastLoggedFrame;
            auto previousFrame = lastFrame.load(std::memory_order_relaxed);
            if ((previousFrame != 0 && target.frameIndex - previousFrame < 90) ||
                !lastFrame.compare_exchange_strong(previousFrame, target.frameIndex, std::memory_order_relaxed)) {
                return;
            }
            ROCK_LOG_DEBUG(HeldScenePresentation,
                "HELD_SCENE_TARGET applied trace={} frame={} hand={} body={} path=writer-target targetStep={:.4f}gu/{:.3f}deg residual={:.4f}gu advance={:.4f}gu writerDelta={:.4f}gu/{:.3f}deg",
                target.traceId,
                target.frameIndex,
                target.isLeft ? "left" : "right",
                bodyId,
                target.targetTranslationStepGameUnits,
                target.targetRotationStepDegrees,
                target.physicalResidualGameUnits,
                target.transportAdvanceGameUnits,
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
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HeldSceneWriter);
            if (!havok_runtime::tryResolveCollisionObjectBody(
                    collisionObject,
                    world,
                    bodyId) ||
                !findExactMatch(
                    collisionObject,
                    world,
                    bodyId.value,
                    match)) {
                profilerTimer.stop();
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

            TargetTransportMatch targetTransport{};
            if (mainCallsite &&
                findTargetTransport(
                    world,
                    bodyId.value,
                    targetTransport)) {
                const float havokToGameScale = physics_scale::havokToGame();
                if (std::isfinite(havokToGameScale) &&
                    havokToGameScale > 0.000001f) {
                    alignas(16) float nativeTarget[
                        held_scene_presentation_policy::
                            kPredictionFloatCount]{};
                    const auto* targetFloats =
                        reinterpret_cast<const float*>(
                            &targetTransport.presentedWorld);
                    for (std::size_t index = 0;
                         index < held_scene_presentation_policy::
                             kPredictionFloatCount;
                         ++index) {
                        nativeTarget[index] = targetFloats[index];
                    }
                    nativeTarget[12] /= havokToGameScale;
                    nativeTarget[13] /= havokToGameScale;
                    nativeTarget[14] /= havokToGameScale;

                    alignas(16) float targetWriterInput[
                        held_scene_presentation_policy::
                            kPredictionFloatCount]{};
                    const auto targetTransform =
                        held_scene_presentation_policy::
                            buildWriterTransform(
                                writerInput,
                                nativeTarget,
                                havokToGameScale,
                                targetWriterInput,
                                kTargetWriterMaxTranslationDeltaGameUnits,
                                kTargetWriterMaxRotationDeltaDegrees);
                    if (targetTransform.apply) {
                        logTargetApplication(
                            targetTransport,
                            bodyId.value,
                            targetTransform,
                            targetWriterInput);
                        profilerTimer.stop();
                        s_originalWriter(
                            collisionObjectRaw,
                            targetWriterInput);
                        return;
                    }
                    ROCK_LOG_SAMPLE_WARN(HeldScenePresentation, 1000,
                        "HELD_SCENE_ASSEMBLY writer rejected trace={} frame={} body={} reason={} delta={:.3f}gu/{:.3f}deg",
                        targetTransport.traceId, targetTransport.frameIndex, bodyId.value,
                        held_scene_presentation_policy::rejectReasonName(targetTransform.reason),
                        targetTransform.translationDeltaGameUnits, targetTransform.rotationDeltaDegrees);
                }
            }

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
                profilerTimer.stop();
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
                profilerTimer.stop();
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
            profilerTimer.stop();
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
        resetTargetTransport(isLeft ? 1u : 0u);
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
        destination.complete.store(registration.complete, std::memory_order_relaxed);
        destination.count.store(count, std::memory_order_relaxed);
        destination.sequence.fetch_add(
            1,
            std::memory_order_release);  // even: complete registration
    }

    void clearHeldBodies(bool isLeft) noexcept
    {
        held_render_trace::clearHand(isLeft);
        Registration empty{};
        publishHeldBodies(isLeft, empty);
    }

    TargetTransportPublication publishTargetTransport(
        bool isLeft,
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint64_t traceId,
        const RE::NiTransform& targetBodyWorld,
        const RE::NiTransform& solvedBodyWorld,
        RE::NiAVObject* referenceRoot,
        const RE::NiTransform& bodyInRoot,
        const RE::NiTransform* resolvedWeaponRoot) noexcept
    {
        const std::size_t handIndex = isLeft ? 1u : 0u;
        if (!world || bodyId == 0x7FFF'FFFFu || traceId == 0) {
            resetTargetTransport(handIndex);
            return {};
        }

        // The existing publication mutex serializes target updates with release
        // and registration reset. The native writer consumes only atomics and
        // never waits on this game-side ownership lock.
        std::scoped_lock publicationLock(s_targetTransportHistoryMutex);
        held_scene_presentation_policy::TargetTransportDecision<
            RE::NiTransform>
            decision{};
        bool rebased = false;
        {
            auto& history = s_targetTransportHistory[handIndex];
            const bool sameIdentity =
                history.valid && history.world == world &&
                history.bodyId == bodyId && history.traceId == traceId;
            if (!sameIdentity) {
                history = TargetTransportHistory{
                    .world = world,
                    .bodyId = bodyId,
                    .traceId = traceId,
                    .previousTargetWorld = targetBodyWorld,
                    .valid = held_scene_presentation_policy::finiteTransform(
                        targetBodyWorld),
                };
                rebased = true;
            } else {
                decision = held_scene_presentation_policy::
                    buildTargetTransport(
                        history.previousTargetWorld,
                        targetBodyWorld,
                        solvedBodyWorld);
                history.previousTargetWorld = targetBodyWorld;
                history.valid =
                    held_scene_presentation_policy::finiteTransform(
                        targetBodyWorld);
            }

            if (resolvedWeaponRoot && referenceRoot &&
                held_scene_presentation_policy::finiteTransform(*resolvedWeaponRoot)) {
                decision.presentedWorld = transform_math::composeTransforms(*resolvedWeaponRoot, bodyInRoot);
                decision.apply = held_scene_presentation_policy::finiteTransform(decision.presentedWorld);
                rebased = false;
            }
            if (rebased || !decision.apply) {
                clearTargetTransportPublication(handIndex);
            }
        }

        if (rebased || !decision.apply) {
            if (!rebased &&
                (g_rockConfig.rockDebugGrabFrameLogging ||
                    g_rockConfig.rockDebugVerboseLogging)) {
                ROCK_LOG_SAMPLE_DEBUG(HeldScenePresentation,
                    1000,
                    "HELD_SCENE_TARGET rejected trace={} hand={} body={} reason={} targetStep={:.4f}gu/{:.3f}deg residual={:.4f}gu advance={:.4f}gu",
                    traceId,
                    isLeft ? "left" : "right",
                    bodyId,
                    held_scene_presentation_policy::
                        targetTransportRejectReasonName(decision.reason),
                    decision.targetTranslationStepGameUnits,
                    decision.targetRotationStepDegrees,
                    decision.physicalResidualGameUnits,
                    decision.transportAdvanceGameUnits);
            }
            return {};
        }

        Registration registration{};
        using ScenePose = held_scene_presentation_policy::ScenePose<RE::NiAVObject, RE::NiTransform>;
        std::array<BodyPose, kMaxRegisteredBodies> poses{};
        std::array<ScenePose, kMaxRegisteredBodies + 1> scenePoses{};
        const char* failure = nullptr;
        std::uint32_t failedBody = bodyId;
        if (!copyRegistration(isLeft, registration) || registration.traceId != traceId ||
            !registration.complete || !containsBody(registration, world, bodyId)) {
            failure = "registration-incomplete-or-changed";
        }
        const auto frameIndex = runtime_state::currentFrame().frameIndex;
        const bool firstAssembly = s_lastAssemblyLogTrace[handIndex] != traceId;
        const bool logBodies = firstAssembly || ((g_rockConfig.rockDebugGrabFrameLogging || g_rockConfig.rockDebugVerboseLogging) &&
            (s_lastAssemblyLogFrame[handIndex] == 0 || frameIndex - s_lastAssemblyLogFrame[handIndex] >= 90));
        if (logBodies) {
            s_lastAssemblyLogFrame[handIndex] = frameIndex;
            s_lastAssemblyLogTrace[handIndex] = traceId;
        }
        for (std::size_t index = 0; !failure && index < registration.count; ++index) {
            const auto& body = registration.bodies[index];
            failedBody = body.bodyId;
            RE::NiTransform solved{};
            auto* collision = havok_runtime::getCollisionObjectFromBody(world, RE::hknpBodyId{body.bodyId});
            if (body.world != world || !collision || collision != body.collisionObject || !collision->sceneObject ||
                !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{body.bodyId}, solved)) {
                failure = "body-or-owner-unavailable";
                break;
            }
            auto& pose = poses[index];
            pose.bodyId = body.bodyId;
            if (body.bodyId == bodyId) {
                pose.world = decision.presentedWorld;
            } else if (resolvedWeaponRoot && referenceRoot) {
                pose.world = transform_math::composeTransforms(*resolvedWeaponRoot,
                    transform_math::composeTransforms(transform_math::invertTransform(referenceRoot->world), collision->sceneObject->world));
                if (!held_scene_presentation_policy::finiteTransform(pose.world)) failure = "physical-part-transform-invalid";
            } else if (!held_scene_presentation_policy::transportAssemblyBody(
                           solvedBodyWorld, decision.presentedWorld, solved, pose.world)) {
                failure = "body-transport-rejected";
                break;
            }
            auto* node = collision->sceneObject;
            scenePoses[index] = {node, pose.world};
            scenePoses[index].world.scale = node->world.scale;
            if (logBodies) {
                ROCK_LOG_INFO(HeldScenePresentation,
                    "HELD_SCENE_ASSEMBLY prepared trace={} frame={} hand={} body={} owner='{}' primary={} output=({:.3f},{:.3f},{:.3f})",
                    traceId, frameIndex, isLeft ? "left" : "right", body.bodyId, node->name.c_str(),
                    body.bodyId == bodyId, pose.world.translate.x, pose.world.translate.y, pose.world.translate.z);
            }
        }
        std::size_t scenePoseCount = registration.count;
        if (!failure && referenceRoot && !held_scene_presentation_policy::appendAssemblyRootPose(
                scenePoses.data(), scenePoseCount, scenePoses.size(), referenceRoot,
                decision.presentedWorld, bodyInRoot)) {
            failure = "reference-root-or-body-ancestry-invalid";
        }
        if (!failure && !held_scene_presentation_policy::prepareScenePoses(scenePoses.data(), scenePoseCount)) {
            failure = "scene-hierarchy-or-alias-conflict";
        }
        std::array<held_scene_presentation_policy::SceneHistoryPose<RE::NiAVObject, RE::NiTransform>, 512> sceneHistory{};
        std::size_t historyCount = 0;
        if (!failure && !held_scene_presentation_policy::captureSceneHistory(scenePoses.data(), scenePoseCount,
                sceneHistory.data(), sceneHistory.size(), historyCount,
                [](RE::NiAVObject* root, auto&& visitor) { return weapon_scene::visitScene(root, visitor); })) {
            failure = "scene-history-incomplete-or-invalid";
        }
        if (failure) {
            clearTargetTransportPublication(handIndex);
            ROCK_LOG_SAMPLE_WARN(HeldScenePresentation, 1000,
                "HELD_SCENE_ASSEMBLY rejected trace={} frame={} hand={} body={} count={} reason={}",
                traceId, frameIndex, isLeft ? "left" : "right", failedBody, registration.count, failure);
            return {};
        }
        publishTargetTransportDecision(handIndex, world, poses.data(), registration.count, traceId, frameIndex, decision);

        TargetTransportMatch selected{};
        if (!findTargetTransport(world, bodyId, selected) || selected.frameIndex != frameIndex) {
            return {};
        }
        if (selected.isLeft == isLeft) {
            // Commit all owner locals first; native geometry updates then see
            // the complete assembly, including mesh-only sibling branches.
            const auto rootBefore = referenceRoot ? referenceRoot->world : RE::NiTransform{};
            held_scene_presentation_policy::applyScenePoses(scenePoses.data(), scenePoseCount,
                [](RE::NiAVObject* node) noexcept { f4vr::updateTransformsDown(node, false); },
                [referenceRoot](RE::NiAVObject* node) noexcept {
                    if (referenceRoot) {
                        // FO4VR BSGeometry::UpdateWorldData (0x1C31C10) also
                        // sets the shader's transform-changed flag. Direct
                        // NiTransform writes alone omit that notification.
                        f4vr::updateDown(node, true);
                    } else {
                        f4vr::updateTransformsDown(node, false);
                    }
                });
            // Native 0x1C23740 snapshots world -> previousWorld before it
            // computes world; BSGeometry 0x1C31C10 calls it. The raw owner
            // writes above have already advanced world, so restore the actual
            // prior presented pose for roots, independent parts and geometry.
            // Native scene writer 0x1E06B00 only writes local/world, not history.
            held_scene_presentation_policy::commitSceneHistory(sceneHistory.data(), historyCount);
            if (logBodies && referenceRoot) {
                ROCK_LOG_INFO(HeldScenePresentation,
                    "HELD_SCENE_ROOT trace={} frame={} hand={} root='{}' bodyOwners={} scenePoses={} advance={:.3f}gu/{:.3f}deg output=({:.3f},{:.3f},{:.3f})",
                    traceId, frameIndex, isLeft ? "left" : "right", referenceRoot->name.c_str(),
                    registration.count, scenePoseCount,
                    held_scene_presentation_policy::pointDistance(rootBefore.translate, referenceRoot->world.translate),
                    held_scene_presentation_policy::matrixRotationDeltaDegrees(rootBefore.rotate, referenceRoot->world.rotate),
                    referenceRoot->world.translate.x, referenceRoot->world.translate.y, referenceRoot->world.translate.z);
            }
        }
        if (logBodies) {
            ROCK_LOG_INFO(HeldScenePresentation,
                "HELD_SCENE_ASSEMBLY committed trace={} frame={} hand={} count={} ownerHand={} path={}",
                traceId, frameIndex, isLeft ? "left" : "right", registration.count,
                selected.isLeft ? "left" : "right", selected.isLeft == isLeft ? "immediate-and-writer" : "peer-publication");
        }
        return TargetTransportPublication{
            .applied = true,
            .presentedBodyWorld = selected.presentedWorld,
        };
    }

    bool refreshPhysicalWeaponParts(bool isLeft, RE::hknpWorld* world, RE::NiAVObject* referenceRoot) noexcept
    {
        if (!world || !referenceRoot) return false;
        std::scoped_lock publicationLock(s_targetTransportHistoryMutex);
        Registration registration{};
        if (!copyRegistration(isLeft, registration) || !registration.complete || !registration.count) return false;
        TargetTransportMatch previous{};
        if (!copyTargetTransport(s_targetTransport[isLeft ? 1u : 0u], isLeft, world, registration.bodies[0].bodyId, previous) ||
            previous.traceId != registration.traceId || previous.frameIndex != runtime_state::currentFrame().frameIndex) return false;
        std::array<BodyPose, kMaxRegisteredBodies> poses{};
        for (std::size_t i = 0; i < registration.count; ++i) {
            const auto& body = registration.bodies[i];
            auto* collision = havok_runtime::getCollisionObjectFromBody(world, RE::hknpBodyId{body.bodyId});
            if (body.world != world || !collision || collision != body.collisionObject || !collision->sceneObject) return false;
            auto* node = collision->sceneObject;
            auto* ancestor = node;
            for (unsigned depth = 0; ancestor && ancestor != referenceRoot && depth < 64; ++depth) ancestor = ancestor->parent;
            if (ancestor != referenceRoot || !held_scene_presentation_policy::finiteTransform(node->world)) return false;
            poses[i] = {body.bodyId, node->world};
        }
        held_scene_presentation_policy::TargetTransportDecision<RE::NiTransform> decision{};
        decision.targetTranslationStepGameUnits = previous.targetTranslationStepGameUnits;
        decision.targetRotationStepDegrees = previous.targetRotationStepDegrees;
        decision.physicalResidualGameUnits = previous.physicalResidualGameUnits;
        decision.transportAdvanceGameUnits = previous.transportAdvanceGameUnits;
        publishTargetTransportDecision(isLeft ? 1u : 0u, world, poses.data(), registration.count,
            registration.traceId, previous.frameIndex, decision);
        return true;
    }

    bool leftOwnsSharedAssembly() noexcept
    {
        Registration right{}, left{};
        if (!copyRegistration(false, right) || !copyRegistration(true, left) ||
            !held_scene_presentation_policy::preferEarlierTrace(left.traceId, right.traceId)) {
            return false;
        }
        for (std::size_t index = 0; index < right.count; ++index) {
            if (containsBody(left, right.bodies[index].world, right.bodies[index].bodyId)) {
                return true;
            }
        }
        return false;
    }

    bool tryGetPresentedBodyWorld(
        RE::hknpWorld* world, std::uint32_t bodyId, std::uint64_t frameIndex,
        RE::NiTransform& outWorld) noexcept
    {
        TargetTransportMatch match{};
        if (!world || frameIndex == 0 || !findTargetTransport(world, bodyId, match) ||
            match.frameIndex != frameIndex) return false;
        outWorld = match.presentedWorld;
        return true;
    }
}
