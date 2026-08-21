#include "physics-interaction/native/SceneWriterProbe.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/hooks/EntryTrampolineHook.h"
#include "physics-interaction/native/havok/HavokOffsets.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"
#include "RE/NetImmerse/NiSmartPointer.h"
#include "REL/Relocation.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <intrin.h>

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
        constexpr float kMaxAnchorRootShiftGameUnits = 35.0f;
        constexpr float kMaxYawDeltaRadians = 0.35f;

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
        std::atomic<std::uint32_t> s_activeHookReaders{ 0 };

        /*
         * All hook-visible fields are atomic. The generation counter adds a
         * coherent multi-field snapshot without relying on a C++ data race.
         * The active flag is published last and cleared first.
         */
        struct alignas(64) HandSlot
        {
            std::atomic<bool> active{ false };
            std::atomic<std::uint32_t> generation{ 0 };
            std::array<std::atomic<const RE::NiCollisionObject*>, kMaxTrackedCollisionObjects>
                collisionObjects{};
            std::atomic<std::uint32_t> collisionObjectCount{ 0 };
            std::atomic<const RE::NiAVObject*> roomNode{ nullptr };
            std::atomic<std::uint64_t> traceId{ 0 };
        };

        HandSlot s_slots[2];
        RE::NiPointer<RE::NiAVObject> s_roomNodeOwners[2];
        RE::NiPointer<RE::NiAVObject> s_retiredRoomNodeOwners[2];

        struct alignas(64) AnchorSlot
        {
            std::atomic<std::uint32_t> generation{ 0 };
            std::array<std::atomic<float>, 12> rotationRows{};
            std::array<std::atomic<float>, 3> translate{};
            std::array<std::atomic<float>, 3> sourceRootPositionHavok{};
            std::atomic<std::uintptr_t> sourceRootControllerIdentity{ 0 };
            std::atomic<float> sourceRootHavokToGame{ 0.0f };
            std::atomic<bool> sourceRootValid{ false };
            std::array<std::atomic<float>, 3> sourceRoomPositionGame{};
            std::atomic<float> sourceRoomYawRadians{ 0.0f };
            std::atomic<bool> sourceRoomValid{ false };
            std::atomic<std::uint8_t> stage{ 0 };
            std::atomic<bool> valid{ false };
        };

        AnchorSlot s_anchorSlots[2];

        struct alignas(64) HookConfigSlot
        {
            std::atomic<std::uint32_t> generation{ 0 };
            std::atomic<bool> syncEnabled{ false };
            std::atomic<float> fullGapGameUnits{ 4.0f };
            std::atomic<float> solverGapGameUnits{ 15.0f };
            std::atomic<float> offsetZGameUnits{ 0.0f };
            std::atomic<bool> valid{ false };
        };

        HookConfigSlot s_hookConfig;

        static_assert(std::atomic<bool>::is_always_lock_free);
        static_assert(std::atomic<float>::is_always_lock_free);
        static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
        static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
        static_assert(
            std::atomic<const RE::NiAVObject*>::is_always_lock_free);
        static_assert(
            std::atomic<const RE::NiCollisionObject*>::is_always_lock_free);

        struct SlotSnapshot
        {
            const RE::NiCollisionObject* collisionObjects[kMaxTrackedCollisionObjects] = {};
            std::uint32_t collisionObjectCount = 0;
            const RE::NiAVObject* roomNode = nullptr;
            std::uint64_t traceId = 0;
        };

        struct AnchorSnapshot
        {
            float rotationRows[12] = {};
            float translate[3] = {};
            AnchorRootSample sourceRoot{};
            std::uint8_t stage = 0;
        };

        struct HookConfigSnapshot
        {
            bool syncEnabled = false;
            float fullGapGameUnits = 4.0f;
            float solverGapGameUnits = 15.0f;
            float offsetZGameUnits = 0.0f;
        };

        class HookReaderGuard
        {
        public:
            HookReaderGuard()
            {
                s_activeHookReaders.fetch_add(1, std::memory_order_acq_rel);
            }

            ~HookReaderGuard()
            {
                s_activeHookReaders.fetch_sub(1, std::memory_order_acq_rel);
            }

            HookReaderGuard(const HookReaderGuard&) = delete;
            HookReaderGuard& operator=(const HookReaderGuard&) = delete;
        };

        bool anyHandActive()
        {
            return s_slots[0].active.load(std::memory_order_acquire) ||
                   s_slots[1].active.load(std::memory_order_acquire);
        }

        bool tryReadSlot(HandSlot& slot, SlotSnapshot& out)
        {
            if (!slot.active.load(std::memory_order_acquire)) {
                return false;
            }
            const std::uint32_t before = slot.generation.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                return false;
            }
            out.collisionObjectCount = slot.collisionObjectCount.load(std::memory_order_relaxed);
            if (out.collisionObjectCount == 0 ||
                out.collisionObjectCount > kMaxTrackedCollisionObjects) {
                return false;
            }
            for (std::uint32_t i = 0; i < out.collisionObjectCount; ++i) {
                out.collisionObjects[i] = slot.collisionObjects[i].load(std::memory_order_relaxed);
            }
            out.roomNode = slot.roomNode.load(std::memory_order_relaxed);
            out.traceId = slot.traceId.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            return slot.generation.load(std::memory_order_acquire) == before &&
                   slot.active.load(std::memory_order_acquire);
        }

        bool tryReadAnchor(AnchorSlot& slot, AnchorSnapshot& out)
        {
            const std::uint32_t before = slot.generation.load(std::memory_order_acquire);
            if ((before & 1u) != 0 || !slot.valid.load(std::memory_order_relaxed)) {
                return false;
            }
            for (std::size_t i = 0; i < 12; ++i) {
                out.rotationRows[i] = slot.rotationRows[i].load(std::memory_order_relaxed);
            }
            for (std::size_t i = 0; i < 3; ++i) {
                out.translate[i] = slot.translate[i].load(std::memory_order_relaxed);
                out.sourceRoot.positionHavok[i] =
                    slot.sourceRootPositionHavok[i].load(std::memory_order_relaxed);
                out.sourceRoot.roomPositionGame[i] =
                    slot.sourceRoomPositionGame[i].load(std::memory_order_relaxed);
            }
            out.sourceRoot.controllerIdentity =
                slot.sourceRootControllerIdentity.load(std::memory_order_relaxed);
            out.sourceRoot.havokToGame =
                slot.sourceRootHavokToGame.load(std::memory_order_relaxed);
            out.sourceRoot.valid = slot.sourceRootValid.load(std::memory_order_relaxed);
            out.sourceRoot.roomYawRadians =
                slot.sourceRoomYawRadians.load(std::memory_order_relaxed);
            out.sourceRoot.roomValid = slot.sourceRoomValid.load(std::memory_order_relaxed);
            out.stage = slot.stage.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            return slot.generation.load(std::memory_order_acquire) == before &&
                   slot.valid.load(std::memory_order_acquire);
        }

        bool tryReadHookConfig(HookConfigSnapshot& out)
        {
            const std::uint32_t before = s_hookConfig.generation.load(std::memory_order_acquire);
            if ((before & 1u) != 0 || !s_hookConfig.valid.load(std::memory_order_relaxed)) {
                return false;
            }
            out.syncEnabled = s_hookConfig.syncEnabled.load(std::memory_order_relaxed);
            out.fullGapGameUnits = s_hookConfig.fullGapGameUnits.load(std::memory_order_relaxed);
            out.solverGapGameUnits = s_hookConfig.solverGapGameUnits.load(std::memory_order_relaxed);
            out.offsetZGameUnits = s_hookConfig.offsetZGameUnits.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            return s_hookConfig.generation.load(std::memory_order_acquire) == before &&
                   s_hookConfig.valid.load(std::memory_order_acquire);
        }

        void publishHookConfig()
        {
            const float configuredFullGap =
                g_rockConfig.rockGrabScenePoseSyncFullAnchorGapGameUnits;
            const float fullGap =
                std::isfinite(configuredFullGap) && configuredFullGap >= 0.0f ?
                    configuredFullGap :
                    4.0f;
            const float configuredSolverGap =
                g_rockConfig.rockGrabScenePoseSyncSolverGapGameUnits;
            const float solverGap =
                std::isfinite(configuredSolverGap) && configuredSolverGap > fullGap ?
                    configuredSolverGap :
                    fullGap + 11.0f;
            const float configuredOffset =
                g_rockConfig.rockGrabSceneWriterProbeOffsetZGameUnits;
            const float offset = std::isfinite(configuredOffset) ? configuredOffset : 0.0f;

            s_hookConfig.generation.fetch_add(1, std::memory_order_acq_rel);
            s_hookConfig.syncEnabled.store(
                g_rockConfig.rockGrabHeldScenePoseSync,
                std::memory_order_relaxed);
            s_hookConfig.fullGapGameUnits.store(fullGap, std::memory_order_relaxed);
            s_hookConfig.solverGapGameUnits.store(solverGap, std::memory_order_relaxed);
            s_hookConfig.offsetZGameUnits.store(offset, std::memory_order_relaxed);
            s_hookConfig.valid.store(true, std::memory_order_relaxed);
            s_hookConfig.generation.fetch_add(1, std::memory_order_release);
        }

        void retireRoomNodeOwnersIfQuiescent()
        {
            if (s_activeHookReaders.load(std::memory_order_acquire) != 0) {
                return;
            }
            for (std::size_t hand = 0; hand < 2; ++hand) {
                s_retiredRoomNodeOwners[hand].reset();
                if (!s_slots[hand].active.load(std::memory_order_acquire)) {
                    s_roomNodeOwners[hand].reset();
                }
            }
        }

        void clearHandSlot(HandSlot& slot)
        {
            slot.active.store(false, std::memory_order_release);
            slot.generation.fetch_add(1, std::memory_order_acq_rel);
            for (auto& collisionObject : slot.collisionObjects) {
                collisionObject.store(nullptr, std::memory_order_relaxed);
            }
            slot.collisionObjectCount.store(0, std::memory_order_relaxed);
            slot.roomNode.store(nullptr, std::memory_order_relaxed);
            slot.traceId.store(0, std::memory_order_relaxed);
            slot.generation.fetch_add(1, std::memory_order_release);
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

        void __fastcall onSceneTransformWriter(void* collisionObject, float* transform)
        {
            const auto original = s_original.load(std::memory_order_relaxed);
            if (!original) {
                return;
            }
            s_totalWriterCalls.fetch_add(1, std::memory_order_relaxed);

            if (!collisionObject || !transform || !anyHandActive()) {
                original(collisionObject, transform);
                return;
            }

            HookReaderGuard readerGuard;
            SlotSnapshot snapshot{};
            int matchedHand = -1;
            for (int hand = 0; hand < 2 && matchedHand < 0; ++hand) {
                SlotSnapshot candidate{};
                if (tryReadSlot(s_slots[hand], candidate) &&
                    snapshotMatches(candidate, collisionObject)) {
                    snapshot = candidate;
                    matchedHand = hand;
                }
            }

            if (matchedHand < 0) {
                original(collisionObject, transform);
                return;
            }

            s_matchedCalls.fetch_add(1, std::memory_order_relaxed);
            const auto returnAddress = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            const auto moduleBase = REL::Module::get().base();
            const std::uintptr_t returnRva =
                returnAddress >= moduleBase ? returnAddress - moduleBase : 0;
            if (returnRva == offsets::kRet_SceneWriterMainCallsite) {
                s_mainCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            } else if (returnRva == offsets::kRet_SceneWriterProxyCallsite) {
                s_proxyCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            } else {
                s_otherCallsiteCalls.fetch_add(1, std::memory_order_relaxed);
            }

            const auto collisionBase = reinterpret_cast<std::uintptr_t>(collisionObject);
            const std::uint8_t collisionFlags =
                *reinterpret_cast<const std::uint8_t*>(collisionBase + 0x18);
            if ((collisionFlags & 0x04) != 0) {
                s_callbackFlagCalls.fetch_add(1, std::memory_order_relaxed);
            }
            if ((collisionFlags & 0x08) != 0) {
                s_localFlagCalls.fetch_add(1, std::memory_order_relaxed);
            }

            HookConfigSnapshot config{};
            const bool hasConfig = tryReadHookConfig(config);
            bool syncApplied = false;
            if (hasConfig && config.syncEnabled) {
                AnchorSnapshot anchor{};
                if (tryReadAnchor(s_anchorSlots[matchedHand], anchor)) {
                    if (anchor.sourceRoot.roomValid && snapshot.roomNode) {
                        const auto& liveRoomWorld = snapshot.roomNode->world;
                        const float liveRoomX = liveRoomWorld.translate.x;
                        const float liveRoomY = liveRoomWorld.translate.y;
                        const float liveRoomZ = liveRoomWorld.translate.z;
                        const float liveYaw = std::atan2(
                            liveRoomWorld.rotate.entry[1][0],
                            liveRoomWorld.rotate.entry[0][0]);
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
                        if (liveRoomFinite && std::fabs(yawDelta) <= kMaxYawDeltaRadians) {
                            const float cosDelta = std::cos(yawDelta);
                            const float sinDelta = std::sin(yawDelta);
                            const float relX =
                                anchor.translate[0] - anchor.sourceRoot.roomPositionGame[0];
                            const float relY =
                                anchor.translate[1] - anchor.sourceRoot.roomPositionGame[1];
                            const float relZ =
                                anchor.translate[2] - anchor.sourceRoot.roomPositionGame[2];
                            const float newX = liveRoomX + cosDelta * relX - sinDelta * relY;
                            const float newY = liveRoomY + sinDelta * relX + cosDelta * relY;
                            const float newZ = liveRoomZ + relZ;
                            const float shiftX = newX - anchor.translate[0];
                            const float shiftY = newY - anchor.translate[1];
                            const float shiftZ = newZ - anchor.translate[2];
                            const float shiftLength =
                                std::sqrt(shiftX * shiftX + shiftY * shiftY + shiftZ * shiftZ);
                            if (std::isfinite(shiftLength) &&
                                shiftLength <= kMaxAnchorRootShiftGameUnits) {
                                anchor.translate[0] = newX;
                                anchor.translate[1] = newY;
                                anchor.translate[2] = newZ;
                                if (std::fabs(yawDelta) > 0.0001f) {
                                    for (int row = 0; row < 3; ++row) {
                                        const float rowX = anchor.rotationRows[row * 4];
                                        const float rowY = anchor.rotationRows[row * 4 + 1];
                                        anchor.rotationRows[row * 4] =
                                            cosDelta * rowX - sinDelta * rowY;
                                        anchor.rotationRows[row * 4 + 1] =
                                            sinDelta * rowX + cosDelta * rowY;
                                    }
                                }
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
                    const float syncGapGameUnits =
                        std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
                    if (std::isfinite(syncGapGameUnits) &&
                        syncGapGameUnits < config.solverGapGameUnits) {
                        const float blend = syncGapGameUnits <= config.fullGapGameUnits ?
                            0.0f :
                            (syncGapGameUnits - config.fullGapGameUnits) /
                                (config.solverGapGameUnits - config.fullGapGameUnits);
                        alignas(16) float substituted[16];
                        std::memcpy(
                            substituted,
                            anchor.rotationRows,
                            sizeof(anchor.rotationRows));
                        substituted[12] =
                            anchor.translate[0] + (transform[12] - anchor.translate[0]) * blend;
                        substituted[13] =
                            anchor.translate[1] + (transform[13] - anchor.translate[1]) * blend;
                        substituted[14] =
                            anchor.translate[2] + (transform[14] - anchor.translate[2]) * blend;
                        substituted[15] = 0.0f;
                        original(collisionObject, substituted);
                        syncApplied = true;
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

            const bool applyOffset =
                !syncApplied && hasConfig && config.offsetZGameUnits != 0.0f;
            if (applyOffset) {
                alignas(16) float substituted[16];
                std::memcpy(substituted, transform, sizeof(float) * kWriterInputFloats);
                substituted[15] = 0.0f;
                substituted[kTranslateZIndex] += config.offsetZGameUnits;
                s_offsetAppliedCalls.fetch_add(1, std::memory_order_relaxed);
                original(collisionObject, substituted);
            } else if (!syncApplied) {
                original(collisionObject, transform);
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
                "Scene-writer boundary unavailable: entry prefix mismatch or trampoline failure at RVA 0x{:X}; engine untouched",
                static_cast<std::uint64_t>(offsets::kFunc_SceneTransformWriter));
            return false;
        }
        s_original.store(reinterpret_cast<SceneWriterFn>(original), std::memory_order_release);
        s_installed.store(true, std::memory_order_release);
        ROCK_LOG_INFO(Init,
            "Scene-writer boundary installed at RVA 0x{:X} (main callsite 0x{:X}, proxy callsite 0x{:X})",
            static_cast<std::uint64_t>(offsets::kFunc_SceneTransformWriter),
            static_cast<std::uint64_t>(offsets::kRet_SceneWriterMainCallsite),
            static_cast<std::uint64_t>(offsets::kRet_SceneWriterProxyCallsite));
        return true;
    }

    bool isInstalled()
    {
        return s_installed.load(std::memory_order_acquire);
    }

    void serviceGameThread()
    {
        retireRoomNodeOwnersIfQuiescent();
    }

    bool registerHeldTarget(bool isLeft, const HeldTargetRegistration& registration)
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return false;
        }

        std::array<const RE::NiCollisionObject*, kMaxTrackedCollisionObjects>
            collisionObjects{};
        std::uint32_t collisionObjectCount = 0;
        const std::uint32_t requestedCount =
            registration.collisionObjectCount > kMaxTrackedCollisionObjects ?
                static_cast<std::uint32_t>(kMaxTrackedCollisionObjects) :
                registration.collisionObjectCount;
        for (std::uint32_t i = 0; i < requestedCount; ++i) {
            const auto* candidate = registration.collisionObjects[i];
            if (!candidate) {
                continue;
            }
            bool duplicate = false;
            for (std::uint32_t existing = 0; existing < collisionObjectCount; ++existing) {
                if (collisionObjects[existing] == candidate) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                collisionObjects[collisionObjectCount++] = candidate;
            }
        }
        if (collisionObjectCount == 0) {
            return false;
        }

        publishHookConfig();
        invalidateHeldAnchor(isLeft);

        const std::size_t handIndex = isLeft ? 1u : 0u;
        auto& slot = s_slots[handIndex];
        clearHandSlot(slot);

        /*
         * Do not replace the strong owner while an earlier matched call can
         * still use its raw room pointer. Registration retries next frame.
         */
        if (s_activeHookReaders.load(std::memory_order_acquire) != 0) {
            return false;
        }
        retireRoomNodeOwnersIfQuiescent();
        s_roomNodeOwners[handIndex] =
            RE::NiPointer<RE::NiAVObject>(registration.roomNode);

        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        for (std::size_t i = 0; i < kMaxTrackedCollisionObjects; ++i) {
            slot.collisionObjects[i].store(collisionObjects[i], std::memory_order_relaxed);
        }
        slot.collisionObjectCount.store(collisionObjectCount, std::memory_order_relaxed);
        slot.roomNode.store(s_roomNodeOwners[handIndex].get(), std::memory_order_relaxed);
        slot.traceId.store(registration.traceId, std::memory_order_relaxed);
        slot.generation.fetch_add(1, std::memory_order_release);
        slot.active.store(true, std::memory_order_release);
        return true;
    }

    void refreshHeldPresentationConfig(const bool isLeft)
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return;
        }
        publishHookConfig();
        retireRoomNodeOwnersIfQuiescent();
        if (!g_rockConfig.rockGrabHeldScenePoseSync) {
            invalidateHeldAnchor(isLeft);
        }
    }

    void clearHeldTarget(bool isLeft)
    {
        invalidateHeldAnchor(isLeft);
        const std::size_t handIndex = isLeft ? 1u : 0u;
        clearHandSlot(s_slots[handIndex]);

        if (s_activeHookReaders.load(std::memory_order_acquire) == 0) {
            retireRoomNodeOwnersIfQuiescent();
            return;
        }

        if (!s_retiredRoomNodeOwners[handIndex]) {
            s_retiredRoomNodeOwners[handIndex] = s_roomNodeOwners[handIndex];
            s_roomNodeOwners[handIndex].reset();
        }
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
        publishHookConfig();
        retireRoomNodeOwnersIfQuiescent();

        static_assert(sizeof(bodyAnchorWorldGame.rotate) == sizeof(float) * 12);
        const auto* rotationRows = reinterpret_cast<const float*>(&bodyAnchorWorldGame.rotate);
        auto& slot = s_anchorSlots[isLeft ? 1 : 0];
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        for (std::size_t i = 0; i < 12; ++i) {
            slot.rotationRows[i].store(rotationRows[i], std::memory_order_relaxed);
        }
        slot.translate[0].store(bodyAnchorWorldGame.translate.x, std::memory_order_relaxed);
        slot.translate[1].store(bodyAnchorWorldGame.translate.y, std::memory_order_relaxed);
        slot.translate[2].store(bodyAnchorWorldGame.translate.z, std::memory_order_relaxed);
        for (std::size_t i = 0; i < 3; ++i) {
            slot.sourceRootPositionHavok[i].store(
                sourceRoot.positionHavok[i],
                std::memory_order_relaxed);
            slot.sourceRoomPositionGame[i].store(
                sourceRoot.roomPositionGame[i],
                std::memory_order_relaxed);
        }
        slot.sourceRootControllerIdentity.store(
            sourceRoot.controllerIdentity,
            std::memory_order_relaxed);
        slot.sourceRootHavokToGame.store(sourceRoot.havokToGame, std::memory_order_relaxed);
        slot.sourceRootValid.store(sourceRoot.valid, std::memory_order_relaxed);
        slot.sourceRoomYawRadians.store(sourceRoot.roomYawRadians, std::memory_order_relaxed);
        slot.sourceRoomValid.store(sourceRoot.roomValid, std::memory_order_relaxed);
        slot.stage.store(static_cast<std::uint8_t>(stage), std::memory_order_relaxed);
        slot.valid.store(true, std::memory_order_relaxed);
        slot.generation.fetch_add(1, std::memory_order_release);
    }

    void invalidateHeldAnchor(bool isLeft)
    {
        auto& slot = s_anchorSlots[isLeft ? 1 : 0];
        if (!slot.valid.load(std::memory_order_acquire)) {
            return;
        }
        slot.generation.fetch_add(1, std::memory_order_acq_rel);
        slot.valid.store(false, std::memory_order_relaxed);
        slot.stage.store(
            static_cast<std::uint8_t>(AnchorStage::None),
            std::memory_order_relaxed);
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
