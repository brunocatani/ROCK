#include "physics-interaction/native/SceneWriterProbe.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/hooks/EntryTrampolineHook.h"
#include "physics-interaction/native/havok/HavokOffsets.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"
#include "REL/Relocation.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <intrin.h>
#include <windows.h>

namespace rock::scene_writer_probe
{
    namespace
    {
        /*
         * Raw-disassembly-verified entry of the physics-transform -> NiAVObject
         * writer at RVA 0x1E06B00 (see the 2026-08-18 dossier). All five
         * instructions are position independent, so the 19-byte prefix is both
         * the live-byte verification signature and the relocated trampoline
         * head.
         */
        constexpr std::array<std::uint8_t, 19> kExpectedWriterPrefix{
            0x4C, 0x8B, 0xDC,                          // mov r11, rsp
            0x49, 0x89, 0x5B, 0x10,                    // mov [r11+0x10], rbx
            0x49, 0x89, 0x6B, 0x18,                    // mov [r11+0x18], rbp
            0x57,                                      // push rdi
            0x48, 0x81, 0xEC, 0x30, 0x01, 0x00, 0x00,  // sub rsp, 0x130
        };

        // Writer input layout (verified): 3x4-float rotation rows at
        // +0x00..+0x2C, translation at +0x30/+0x34/+0x38. 15 floats consumed.
        constexpr std::size_t kWriterInputFloats = 15;
        constexpr std::size_t kTranslateXIndex = 12;
        constexpr std::size_t kTranslateZIndex = 14;

        using SceneWriterFn = void(__fastcall*)(void* collisionObject, float* transform);

        std::atomic<SceneWriterFn> s_original{ nullptr };
        std::atomic<bool> s_installed{ false };
        std::atomic<bool> s_installAttempted{ false };

        std::atomic<std::uint64_t> s_totalWriterCalls{ 0 };
        std::atomic<std::uint64_t> s_matchedCalls{ 0 };
        std::atomic<std::uint64_t> s_mainCallsiteCalls{ 0 };
        std::atomic<std::uint64_t> s_proxyCallsiteCalls{ 0 };
        std::atomic<std::uint64_t> s_otherCallsiteCalls{ 0 };
        std::atomic<std::uint64_t> s_offsetAppliedCalls{ 0 };
        std::atomic<std::uint64_t> s_callbackFlagCalls{ 0 };
        std::atomic<std::uint64_t> s_localFlagCalls{ 0 };
        std::atomic<std::uint64_t> s_syncAppliedCalls{ 0 };
        std::atomic<std::uint64_t> s_syncProducerStageCalls{ 0 };
        std::atomic<std::uint64_t> s_syncPreFrikStageCalls{ 0 };
        std::atomic<std::uint64_t> s_syncDivergenceSkips{ 0 };
        std::atomic<std::uint64_t> s_syncRebasedCalls{ 0 };
        std::atomic<std::uint64_t> s_syncRebaseSkips{ 0 };

        /*
         * Seqlock-style per-hand slot: the game thread publishes with
         * generation odd->write->even; the hook takes one stable snapshot or
         * skips. A missed match during the two-store window only delays the
         * next capture by one writer call; it can never tear a pointer read.
         */
        struct alignas(64) HandSlot
        {
            std::atomic<std::uint32_t> generation{ 0 };
            const RE::NiCollisionObject* collisionObjects[kMaxTrackedCollisionObjects] = {};
            std::uint32_t collisionObjectCount = 0;
            RE::hknpWorld* world = nullptr;
            const RE::NiAVObject* roomNode = nullptr;
            std::uint32_t bodyId = 0x7FFF'FFFF;
            float havokToGame = 0.0f;
            std::uint64_t traceId = 0;
            std::atomic<std::uint64_t> lastFullLogTraceId{ 0 };
        };

        HandSlot s_slots[2];

        /*
         * The anchor gets its own seqlock so the twice-per-frame game-thread
         * publish never invalidates the rarely-written registration slot.
         * Layout matches the writer input: 3 rows of 4 floats, then translate.
         */
        struct alignas(64) AnchorSlot
        {
            std::atomic<std::uint32_t> generation{ 0 };
            float rotationRows[12] = {};
            float translate[3] = {};
            AnchorRootSample sourceRoot{};
            std::uint8_t stage = 0;
            bool valid = false;
        };

        AnchorSlot s_anchorSlots[2];

        struct AnchorSnapshot
        {
            float rotationRows[12] = {};
            float translate[3] = {};
            AnchorRootSample sourceRoot{};
            std::uint8_t stage = 0;
        };

        bool tryReadAnchor(AnchorSlot& slot, AnchorSnapshot& out)
        {
            const std::uint32_t before = slot.generation.load(std::memory_order_acquire);
            if ((before & 1u) != 0 || !slot.valid) {
                return false;
            }
            std::memcpy(out.rotationRows, slot.rotationRows, sizeof(out.rotationRows));
            out.translate[0] = slot.translate[0];
            out.translate[1] = slot.translate[1];
            out.translate[2] = slot.translate[2];
            out.sourceRoot = slot.sourceRoot;
            out.stage = slot.stage;
            std::atomic_thread_fence(std::memory_order_acquire);
            return slot.generation.load(std::memory_order_acquire) == before;
        }

        // A one-frame root step above this is a teleport/recenter, not motion.
        constexpr float kMaxAnchorRootShiftGameUnits = 35.0f;

        struct SlotSnapshot
        {
            const RE::NiCollisionObject* collisionObjects[kMaxTrackedCollisionObjects] = {};
            std::uint32_t collisionObjectCount = 0;
            RE::hknpWorld* world = nullptr;
            const RE::NiAVObject* roomNode = nullptr;
            std::uint32_t bodyId = 0x7FFF'FFFF;
            float havokToGame = 0.0f;
            std::uint64_t traceId = 0;
        };

        bool tryReadSlot(HandSlot& slot, SlotSnapshot& out)
        {
            const std::uint32_t before = slot.generation.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                return false;
            }
            out.collisionObjectCount = slot.collisionObjectCount;
            if (out.collisionObjectCount > kMaxTrackedCollisionObjects) {
                return false;
            }
            for (std::uint32_t i = 0; i < out.collisionObjectCount; ++i) {
                out.collisionObjects[i] = slot.collisionObjects[i];
            }
            out.world = slot.world;
            out.roomNode = slot.roomNode;
            out.bodyId = slot.bodyId;
            out.havokToGame = slot.havokToGame;
            out.traceId = slot.traceId;
            std::atomic_thread_fence(std::memory_order_acquire);
            return slot.generation.load(std::memory_order_acquire) == before;
        }

        bool snapshotMatches(const SlotSnapshot& snapshot, const void* collisionObject)
        {
            for (std::uint32_t i = 0; i < snapshot.collisionObjectCount; ++i) {
                if (snapshot.collisionObjects[i] == collisionObject) {
                    return true;
                }
            }
            return false;
        }

        struct MotionSample
        {
            float centerHavok[3] = { 0.0f, 0.0f, 0.0f };
            bool valid = false;
        };

        /*
         * Verified layout walk (dossier + hknpBody.h): bodies at
         * [world+0x20] stride 0x90, motion index at body+0x68, motions at
         * [world+0xE0] stride 0x80, current center at motion+0x00. Every hop
         * is plausibility gated and fails closed to an invalid sample.
         */
        MotionSample readMotionCenter(RE::hknpWorld* world, std::uint32_t bodyId)
        {
            MotionSample sample{};
            if (!world || bodyId == 0x7FFF'FFFF) {
                return sample;
            }
            const auto worldBase = reinterpret_cast<std::uintptr_t>(world);
            const auto bodyArray = *reinterpret_cast<std::uintptr_t*>(worldBase + 0x20);
            const auto motionArray = *reinterpret_cast<std::uintptr_t*>(worldBase + 0xE0);
            if (bodyArray == 0 || motionArray == 0 || bodyId > 0x000F'FFFF) {
                return sample;
            }
            const auto body = bodyArray + static_cast<std::uintptr_t>(bodyId) * 0x90;
            const auto storedBodyId = *reinterpret_cast<std::uint32_t*>(body + 0x60);
            if ((storedBodyId & 0x7FFF'FFFF) != bodyId) {
                return sample;
            }
            const auto motionId = *reinterpret_cast<std::uint32_t*>(body + 0x68);
            if (motionId == 0 || motionId > 0x000F'FFFF) {
                return sample;
            }
            const auto motion = motionArray + static_cast<std::uintptr_t>(motionId) * 0x80;
            const float* center = reinterpret_cast<const float*>(motion + 0x00);
            if (!std::isfinite(center[0]) || !std::isfinite(center[1]) || !std::isfinite(center[2])) {
                return sample;
            }
            sample.centerHavok[0] = center[0];
            sample.centerHavok[1] = center[1];
            sample.centerHavok[2] = center[2];
            sample.valid = true;
            return sample;
        }

        void __fastcall onSceneTransformWriter(void* collisionObject, float* transform)
        {
            const auto original = s_original.load(std::memory_order_relaxed);
            if (!original) {
                return;
            }
            s_totalWriterCalls.fetch_add(1, std::memory_order_relaxed);

            SlotSnapshot snapshot{};
            int matchedHand = -1;
            for (int hand = 0; hand < 2 && matchedHand < 0; ++hand) {
                if (s_slots[hand].collisionObjectCount == 0 &&
                    (s_slots[hand].generation.load(std::memory_order_relaxed) & 1u) == 0) {
                    continue;
                }
                SlotSnapshot candidate{};
                if (tryReadSlot(s_slots[hand], candidate) && snapshotMatches(candidate, collisionObject)) {
                    snapshot = candidate;
                    matchedHand = hand;
                }
            }

            if (matchedHand < 0 || !collisionObject || !transform) {
                original(collisionObject, transform);
                return;
            }

            s_matchedCalls.fetch_add(1, std::memory_order_relaxed);

            const auto returnAddress = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            const auto moduleBase = REL::Module::get().base();
            const std::uintptr_t returnRva = returnAddress >= moduleBase ? returnAddress - moduleBase : 0;
            const char* callsite = "other";
            if (returnRva == offsets::kRet_SceneWriterMainCallsite) {
                callsite = "main";
                s_mainCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            } else if (returnRva == offsets::kRet_SceneWriterProxyCallsite) {
                callsite = "proxy";
                s_proxyCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            } else {
                s_otherCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            }

            const auto collisionBase = reinterpret_cast<std::uintptr_t>(collisionObject);
            const auto vptr = *reinterpret_cast<std::uintptr_t*>(collisionBase);
            const std::uint8_t collisionFlags = *reinterpret_cast<const std::uint8_t*>(collisionBase + 0x18);
            if ((collisionFlags & 0x04) != 0) {
                s_callbackFlagCalls.fetch_add(1, std::memory_order_relaxed);
            }
            if ((collisionFlags & 0x08) != 0) {
                s_localFlagCalls.fetch_add(1, std::memory_order_relaxed);
            }

            auto* sceneObject = static_cast<const RE::NiCollisionObject*>(collisionObject)->sceneObject;
            float nodeBefore[3] = { 0.0f, 0.0f, 0.0f };
            if (sceneObject) {
                nodeBefore[0] = sceneObject->world.translate.x;
                nodeBefore[1] = sceneObject->world.translate.y;
                nodeBefore[2] = sceneObject->world.translate.z;
            }
            const MotionSample motion = readMotionCenter(snapshot.world, snapshot.bodyId);

            float inputTranslate[3] = {
                transform[kTranslateXIndex],
                transform[kTranslateXIndex + 1],
                transform[kTranslateZIndex],
            };

            /*
             * Contract A render-pose sync: full anchor pose below the full-gap
             * threshold, translation blended toward the solver up to the
             * solver-gap threshold (rotation stays on the anchor while any
             * blend applies), untouched solver pose beyond it. The diagnostic
             * offset only runs when the sync did not substitute, so there is
             * exactly one visual authority per call.
             */
            float syncGapGameUnits = -1.0f;
            float syncRootShiftGameUnits = -1.0f;
            std::uint8_t syncStage = 0;
            bool syncApplied = false;
            if (g_rockConfig.rockGrabHeldScenePoseSync) {
                AnchorSnapshot anchor{};
                if (tryReadAnchor(s_anchorSlots[matchedHand], anchor)) {
                    /*
                     * Write-time root rebase: the writer consumes the
                     * producer-stage anchor before ROCK's pre-FRIK refresh
                     * runs, so the anchor is one mid-frame locomotion step
                     * stale. Sample the live controller root (SEH-guarded,
                     * fail-closed) and carry the anchor by the measured step.
                     * Identity/plausibility gates fail closed to the
                     * unshifted anchor; the divergence gate below still owns
                     * the final decision.
                     */
                    /*
                     * Primary rebase: live ROOM node vs the anchor's source
                     * room frame. Stick locomotion moves the room node, and
                     * the camera inherits it; the character controller was
                     * measured unmoved at draw time (rootShift=0, 13:01
                     * session), so it stays telemetry-only below. Rigid 2D
                     * room delta: rotate about the source room origin by the
                     * yaw delta, then carry by the room translation. Snap
                     * turns and teleports are gated out; the divergence gate
                     * below still owns the final decision.
                     */
                    if (anchor.sourceRoot.roomValid && snapshot.roomNode) {
                        const auto& liveRoomWorld = snapshot.roomNode->world;
                        const float liveRoomX = liveRoomWorld.translate.x;
                        const float liveRoomY = liveRoomWorld.translate.y;
                        const float liveRoomZ = liveRoomWorld.translate.z;
                        const float liveYaw =
                            std::atan2(liveRoomWorld.rotate.entry[1][0], liveRoomWorld.rotate.entry[0][0]);
                        float yawDelta = liveYaw - anchor.sourceRoot.roomYawRadians;
                        while (yawDelta > 3.14159265f) {
                            yawDelta -= 6.2831853f;
                        }
                        while (yawDelta < -3.14159265f) {
                            yawDelta += 6.2831853f;
                        }
                        const bool liveRoomFinite =
                            std::isfinite(liveRoomX) && std::isfinite(liveRoomY) &&
                            std::isfinite(liveRoomZ) && std::isfinite(yawDelta);
                        constexpr float kMaxYawDeltaRadians = 0.35f;
                        if (liveRoomFinite && std::fabs(yawDelta) <= kMaxYawDeltaRadians) {
                            const float cosDelta = std::cos(yawDelta);
                            const float sinDelta = std::sin(yawDelta);
                            const float relX = anchor.translate[0] - anchor.sourceRoot.roomPositionGame[0];
                            const float relY = anchor.translate[1] - anchor.sourceRoot.roomPositionGame[1];
                            const float relZ = anchor.translate[2] - anchor.sourceRoot.roomPositionGame[2];
                            const float newX = liveRoomX + cosDelta * relX - sinDelta * relY;
                            const float newY = liveRoomY + sinDelta * relX + cosDelta * relY;
                            const float newZ = liveRoomZ + relZ;
                            const float shiftX = newX - anchor.translate[0];
                            const float shiftY = newY - anchor.translate[1];
                            const float shiftZ = newZ - anchor.translate[2];
                            const float shiftLength =
                                std::sqrt(shiftX * shiftX + shiftY * shiftY + shiftZ * shiftZ);
                            if (std::isfinite(shiftLength) && shiftLength <= kMaxAnchorRootShiftGameUnits) {
                                anchor.translate[0] = newX;
                                anchor.translate[1] = newY;
                                anchor.translate[2] = newZ;
                                if (std::fabs(yawDelta) > 0.0001f) {
                                    // Rotate each stored rotation row's XY by
                                    // the yaw delta. Walk has yawDelta ~0;
                                    // turn correctness is verified visually
                                    // (a wrong convention shows as the held
                                    // object counter-rotating on smooth turn).
                                    for (int row = 0; row < 3; ++row) {
                                        const float rowX = anchor.rotationRows[row * 4];
                                        const float rowY = anchor.rotationRows[row * 4 + 1];
                                        anchor.rotationRows[row * 4] = cosDelta * rowX - sinDelta * rowY;
                                        anchor.rotationRows[row * 4 + 1] = sinDelta * rowX + cosDelta * rowY;
                                    }
                                }
                                syncRootShiftGameUnits = shiftLength;
                                s_syncRebasedCalls.fetch_add(1, std::memory_order_relaxed);
                            } else {
                                s_syncRebaseSkips.fetch_add(1, std::memory_order_relaxed);
                            }
                        } else {
                            s_syncRebaseSkips.fetch_add(1, std::memory_order_relaxed);
                        }
                    }
                    const float deltaX = anchor.translate[0] - transform[kTranslateXIndex];
                    const float deltaY = anchor.translate[1] - transform[kTranslateXIndex + 1];
                    const float deltaZ = anchor.translate[2] - transform[kTranslateZIndex];
                    syncGapGameUnits = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
                    const float fullGapConfig = g_rockConfig.rockGrabScenePoseSyncFullAnchorGapGameUnits;
                    const float solverGapConfig = g_rockConfig.rockGrabScenePoseSyncSolverGapGameUnits;
                    const float fullGap = std::isfinite(fullGapConfig) && fullGapConfig >= 0.0f ? fullGapConfig : 4.0f;
                    const float solverGap =
                        std::isfinite(solverGapConfig) && solverGapConfig > fullGap ? solverGapConfig : fullGap + 11.0f;
                    if (std::isfinite(syncGapGameUnits) && syncGapGameUnits < solverGap) {
                        const float blend =
                            syncGapGameUnits <= fullGap ? 0.0f : (syncGapGameUnits - fullGap) / (solverGap - fullGap);
                        alignas(16) float substituted[16];
                        std::memcpy(substituted, anchor.rotationRows, sizeof(anchor.rotationRows));
                        substituted[12] = anchor.translate[0] + (transform[12] - anchor.translate[0]) * blend;
                        substituted[13] = anchor.translate[1] + (transform[13] - anchor.translate[1]) * blend;
                        substituted[14] = anchor.translate[2] + (transform[14] - anchor.translate[2]) * blend;
                        substituted[15] = 0.0f;
                        original(collisionObject, substituted);
                        syncApplied = true;
                        syncStage = anchor.stage;
                        s_syncAppliedCalls.fetch_add(1, std::memory_order_relaxed);
                        if (anchor.stage == static_cast<std::uint8_t>(AnchorStage::PreFrik)) {
                            s_syncPreFrikStageCalls.fetch_add(1, std::memory_order_relaxed);
                        } else {
                            s_syncProducerStageCalls.fetch_add(1, std::memory_order_relaxed);
                        }
                    } else {
                        s_syncDivergenceSkips.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }

            const float offsetZ = g_rockConfig.rockGrabSceneWriterProbeOffsetZGameUnits;
            const bool applyOffset = !syncApplied && std::isfinite(offsetZ) && offsetZ != 0.0f;
            if (applyOffset) {
                alignas(16) float substituted[16];
                std::memcpy(substituted, transform, sizeof(float) * kWriterInputFloats);
                substituted[15] = 0.0f;
                substituted[kTranslateZIndex] += offsetZ;
                s_offsetAppliedCalls.fetch_add(1, std::memory_order_relaxed);
                original(collisionObject, substituted);
            } else if (!syncApplied) {
                original(collisionObject, transform);
            }

            float nodeAfter[3] = { 0.0f, 0.0f, 0.0f };
            if (sceneObject) {
                nodeAfter[0] = sceneObject->world.translate.x;
                nodeAfter[1] = sceneObject->world.translate.y;
                nodeAfter[2] = sceneObject->world.translate.z;
            }

            /*
             * One unconditional full snapshot per grab (trace id), then the
             * shared sample interval. The hook thread is unknown, so only the
             * bounded formatter below runs here; no allocation, no locks.
             */
            auto& slot = s_slots[matchedHand];
            const std::uint64_t lastLogged = slot.lastFullLogTraceId.load(std::memory_order_relaxed);
            const bool firstForGrab = snapshot.traceId != 0 && lastLogged != snapshot.traceId;
            if (firstForGrab) {
                slot.lastFullLogTraceId.store(snapshot.traceId, std::memory_order_relaxed);
            }
            const float havokToGame = snapshot.havokToGame;
            const float motionGameX = motion.valid ? motion.centerHavok[0] * havokToGame : -1.0f;
            const float motionGameY = motion.valid ? motion.centerHavok[1] * havokToGame : -1.0f;
            const float motionGameZ = motion.valid ? motion.centerHavok[2] * havokToGame : -1.0f;
            if (firstForGrab) {
                ROCK_LOG_INFO(Hand,
                    "SCENE_WRITER first-hit hand={} site={} retRva=0x{:X} vptr=0x{:X} flags=0x{:02X} body={} in=({:.2f},{:.2f},{:.2f}) nodeBefore=({:.2f},{:.2f},{:.2f}) nodeAfter=({:.2f},{:.2f},{:.2f}) motionGame=({:.2f},{:.2f},{:.2f}) sync={} stage={} gap={:.3f} offZ={:.1f} thread={}",
                    matchedHand == 0 ? "R" : "L",
                    callsite,
                    returnRva,
                    vptr - moduleBase,
                    collisionFlags,
                    snapshot.bodyId,
                    inputTranslate[0], inputTranslate[1], inputTranslate[2],
                    nodeBefore[0], nodeBefore[1], nodeBefore[2],
                    nodeAfter[0], nodeAfter[1], nodeAfter[2],
                    motionGameX, motionGameY, motionGameZ,
                    syncApplied ? "yes" : "no",
                    syncStage,
                    syncGapGameUnits,
                    applyOffset ? offsetZ : 0.0f,
                    ::GetCurrentThreadId());
            } else {
                ROCK_LOG_SAMPLE_INFO(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "SCENE_WRITER hit hand={} site={} flags=0x{:02X} in=({:.2f},{:.2f},{:.2f}) nodeAfter=({:.2f},{:.2f},{:.2f}) motionGame=({:.2f},{:.2f},{:.2f}) inVsMotion={:.3f} sync={} stage={} gap={:.3f} roomShift={:.3f} syncApplied={} prodStage={} preFrikStage={} divergeSkips={} rebased={} rebaseSkips={} offZ={:.1f} matched={} thread={}",
                    matchedHand == 0 ? "R" : "L",
                    callsite,
                    collisionFlags,
                    inputTranslate[0], inputTranslate[1], inputTranslate[2],
                    nodeAfter[0], nodeAfter[1], nodeAfter[2],
                    motionGameX, motionGameY, motionGameZ,
                    motion.valid ? std::sqrt(
                        (inputTranslate[0] - motionGameX) * (inputTranslate[0] - motionGameX) +
                        (inputTranslate[1] - motionGameY) * (inputTranslate[1] - motionGameY) +
                        (inputTranslate[2] - motionGameZ) * (inputTranslate[2] - motionGameZ)) : -1.0f,
                    syncApplied ? "yes" : "no",
                    syncStage,
                    syncGapGameUnits,
                    syncRootShiftGameUnits,
                    s_syncAppliedCalls.load(std::memory_order_relaxed),
                    s_syncProducerStageCalls.load(std::memory_order_relaxed),
                    s_syncPreFrikStageCalls.load(std::memory_order_relaxed),
                    s_syncDivergenceSkips.load(std::memory_order_relaxed),
                    s_syncRebasedCalls.load(std::memory_order_relaxed),
                    s_syncRebaseSkips.load(std::memory_order_relaxed),
                    applyOffset ? offsetZ : 0.0f,
                    s_matchedCalls.load(std::memory_order_relaxed),
                    ::GetCurrentThreadId());
            }
        }
    }

    bool install()
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (s_installAttempted.exchange(true, std::memory_order_acq_rel)) {
            return s_installed.load(std::memory_order_acquire);
        }

        void* original = nullptr;
        const bool installed = entry_trampoline_hook::install(
            "physics-to-scene transform writer probe",
            offsets::kFunc_SceneTransformWriter,
            kExpectedWriterPrefix.data(),
            kExpectedWriterPrefix.size(),
            reinterpret_cast<void*>(&onSceneTransformWriter),
            original);
        if (!installed || !original) {
            ROCK_LOG_ERROR(Init,
                "Scene-writer probe unavailable: entry prefix mismatch or trampoline failure at RVA 0x{:X}; engine untouched",
                static_cast<std::uint64_t>(offsets::kFunc_SceneTransformWriter));
            return false;
        }
        s_original.store(reinterpret_cast<SceneWriterFn>(original), std::memory_order_release);
        s_installed.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init,
            "Scene-writer probe installed at RVA 0x{:X} (main callsite 0x{:X}, proxy callsite 0x{:X})",
            static_cast<std::uint64_t>(offsets::kFunc_SceneTransformWriter),
            static_cast<std::uint64_t>(offsets::kRet_SceneWriterMainCallsite),
            static_cast<std::uint64_t>(offsets::kRet_SceneWriterProxyCallsite));
        return true;
    }

    bool isInstalled()
    {
        return s_installed.load(std::memory_order_acquire);
    }

    void registerHeldTarget(bool isLeft, const HeldTargetRegistration& registration)
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return;
        }
        auto& slot = s_slots[isLeft ? 1 : 0];
        const std::uint32_t count =
            registration.collisionObjectCount > kMaxTrackedCollisionObjects ?
                static_cast<std::uint32_t>(kMaxTrackedCollisionObjects) :
                registration.collisionObjectCount;
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        for (std::uint32_t i = 0; i < kMaxTrackedCollisionObjects; ++i) {
            slot.collisionObjects[i] = i < count ? registration.collisionObjects[i] : nullptr;
        }
        slot.collisionObjectCount = count;
        slot.world = registration.world;
        slot.roomNode = registration.roomNode;
        slot.bodyId = registration.bodyId;
        slot.havokToGame = registration.havokToGame;
        slot.traceId = registration.traceId;
        slot.generation.fetch_add(1, std::memory_order_release);
        ROCK_LOG_INFO(Hand,
            "SCENE_WRITER target registered hand={} collisionObjects={} body={} traceId={}",
            isLeft ? "L" : "R",
            count,
            registration.bodyId,
            registration.traceId);
    }

    void clearHeldTarget(bool isLeft)
    {
        invalidateHeldAnchor(isLeft);
        auto& slot = s_slots[isLeft ? 1 : 0];
        if (slot.collisionObjectCount == 0 && slot.traceId == 0) {
            return;
        }
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        for (auto& pointer : slot.collisionObjects) {
            pointer = nullptr;
        }
        slot.collisionObjectCount = 0;
        slot.world = nullptr;
        slot.roomNode = nullptr;
        slot.bodyId = 0x7FFF'FFFF;
        slot.havokToGame = 0.0f;
        slot.traceId = 0;
        slot.generation.fetch_add(1, std::memory_order_release);
    }

    void publishHeldAnchor(
        bool isLeft,
        const RE::NiTransform& bodyAnchorWorldGame,
        AnchorStage stage,
        const AnchorRootSample& sourceRoot)
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return;
        }
        static_assert(sizeof(bodyAnchorWorldGame.rotate) == sizeof(float) * 12);
        auto& slot = s_anchorSlots[isLeft ? 1 : 0];
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        std::memcpy(slot.rotationRows, &bodyAnchorWorldGame.rotate, sizeof(slot.rotationRows));
        slot.translate[0] = bodyAnchorWorldGame.translate.x;
        slot.translate[1] = bodyAnchorWorldGame.translate.y;
        slot.translate[2] = bodyAnchorWorldGame.translate.z;
        slot.sourceRoot = sourceRoot;
        slot.stage = static_cast<std::uint8_t>(stage);
        slot.valid = true;
        slot.generation.fetch_add(1, std::memory_order_release);
    }

    void invalidateHeldAnchor(bool isLeft)
    {
        auto& slot = s_anchorSlots[isLeft ? 1 : 0];
        if (!slot.valid) {
            return;
        }
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        slot.valid = false;
        slot.stage = static_cast<std::uint8_t>(AnchorStage::None);
        slot.generation.fetch_add(1, std::memory_order_release);
    }

    void copyStatus(Status& out)
    {
        out.matchedCalls = s_matchedCalls.load(std::memory_order_relaxed);
        out.mainCallsiteCalls = s_mainCallsiteCalls.load(std::memory_order_relaxed);
        out.proxyCallsiteCalls = s_proxyCallsiteCalls.load(std::memory_order_relaxed);
        out.otherCallsiteCalls = s_otherCallsiteCalls.load(std::memory_order_relaxed);
        out.offsetAppliedCalls = s_offsetAppliedCalls.load(std::memory_order_relaxed);
        out.callbackFlagCalls = s_callbackFlagCalls.load(std::memory_order_relaxed);
        out.localFlagCalls = s_localFlagCalls.load(std::memory_order_relaxed);
        out.totalWriterCalls = s_totalWriterCalls.load(std::memory_order_relaxed);
        out.syncAppliedCalls = s_syncAppliedCalls.load(std::memory_order_relaxed);
        out.syncProducerStageCalls = s_syncProducerStageCalls.load(std::memory_order_relaxed);
        out.syncPreFrikStageCalls = s_syncPreFrikStageCalls.load(std::memory_order_relaxed);
        out.syncDivergenceSkips = s_syncDivergenceSkips.load(std::memory_order_relaxed);
        out.syncRebasedCalls = s_syncRebasedCalls.load(std::memory_order_relaxed);
        out.syncRebaseSkips = s_syncRebaseSkips.load(std::memory_order_relaxed);
        out.installed = s_installed.load(std::memory_order_acquire);
    }
}
