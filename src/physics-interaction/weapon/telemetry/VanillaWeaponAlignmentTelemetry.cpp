#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativeWeaponFadeSuppression.h"
#include "api/ROCKProviderApiInternal.h"

#include "RockConfig.h"
#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"

#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string_view>
#include <Windows.h>

namespace rock::vanilla_weapon_alignment_telemetry
{
    namespace
    {
        struct TransferSourceFrame
        {
            std::uint64_t frame{};
            std::uintptr_t model{}; // Comparison only; never dereferenced.
            RE::NiTransform modelWorld{}, handWorld{}, renderedHandWorld{};
            bool handValid{}, renderedHandValid{};
        };
        struct TransferTrace
        {
            std::uint64_t id{}, startFrame{}, terminalFrame{}, targetFrame{};
            TransferKind kind{};
            std::uint32_t sourceId{};
            std::uintptr_t targetModel{};
            RE::NiTransform targetWorld{};
            // Held-equip continuity investigation. Values and validated scene
            // paths only; no scene leases or delayed raw-pointer dereferences.
            std::chrono::steady_clock::time_point startedAt{};
            std::uint32_t expectedForm{};
            std::uint64_t bridgeFrame{}, finalFrame{};
            std::uintptr_t bridgeModel{}, previousNativeModel{};
            RE::NiTransform bridgeWorld{};
            equipped_weapon_visual_state::ObservationCache nativeCache{};
            bool gapWindowEnded{};
        };
        constexpr std::uint64_t kEquipGapFrames = 240;
        // Investigation owner: ROCK weapon presentation. Remove this targeted
        // capture once 10mm scale, SMG translation and pipe rotation are qualified.
        // Existing bDebugWeaponOmodDump gates it and defaults to false. Enable
        // before skeleton creation. No worker or file exists when disabled.
        struct Session
        {
            // Declaration order releases the logger before its dedicated pool.
            // The bounded queue never waits for disk space; overruns are logged.
            std::shared_ptr<spdlog::details::thread_pool> pool;
            std::shared_ptr<spdlog::async_logger> log;
            std::chrono::steady_clock::time_point lastSample{};
            std::uint64_t sequence{ 0 };
            std::uint64_t cycleTraceSequence{ 0 };
            std::uint32_t formId{ 0 };
            bool sampling{ false };
            unsigned int nativeMask{ 0 };
            std::uint64_t captureFailures{ 0 };
            std::array<std::chrono::steady_clock::time_point, 2> lastLooseSample{};
            std::uint32_t aimCapturesRemaining{ 0 };
            std::chrono::steady_clock::time_point lastAimJump{};
            // Scope direction/roll investigation: remove after entry-pose
            // calibration is qualified. Values/identity tokens only, no held nodes.
            std::chrono::steady_clock::time_point lastScopeSample{};
            std::uint64_t scopeFrame{ 0 };
            std::uint64_t scopeGeneration{ 0 };
            std::uint32_t scopeFormId{ 0 };
            std::uintptr_t scopeWeaponIdentity{ 0 }, scopeCameraIdentity{ 0 };
            RE::NiTransform scopeCameraWeaponLocal{};
            // Owner: moving weapon transfer investigation. Remove when both
            // transfer directions are qualified. No new gate, worker or scene lease.
            std::uint64_t nextTransferTrace{};
            std::array<TransferTrace, 2> transfers{};
            std::array<TransferSourceFrame, 2> lastTransferSource{};
            // Scope hand presentation investigation: four frames at each edge,
            // using the existing diagnostic gate and bounded async writer.
            std::uint64_t scopeHandEdgeFrame{};
            bool scopeHandStateKnown{}, scopeHandOpen{};
            // Owner: native melee/equip fade investigation. Remove after the
            // first-hit -> later-equip transition is qualified. Values only;
            // the existing bounded writer owns all I/O and no scene objects.
            bool meleeFadeReady{};
            std::array<ULONGLONG, 2> meleeFadeNextSample{};
            std::uint64_t meleeFadeWrites{};
            float meleeFadeRequested{ std::numeric_limits<float>::quiet_NaN() };
            std::uintptr_t meleeFadeNativeRoot{};
        };
        std::unique_ptr<Session> session;
        // Lifecycle and phase capture share the game thread. Native animation
        // workers see false in their own TLS and never read the session pointer.
        thread_local bool captureThread = false;

        using NativeFadeWrite = std::uint32_t (*)(RE::NiAVObject*, const float*);
        // Process-lifetime hook. Skeleton shutdown stops observation through
        // captureThread/session; the original call still runs with identical args.
        NativeFadeWrite originalFadeWrite = nullptr;

        template <std::size_t N>
        bool fadeBytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }

        bool validateMeleeFadeLayout()
        {
            // FO4VR raw disassembly, 2026-10-02. Impact 140EFFA81/A8A writes
            // player+908/90C; 140EF7E35 reads them, 140EF7EB7 caches at +910.
            // 140F4B270 and 1427AB9D0 agree on geometry+178 -> shader+48.
            // 140F4B2B9/C2 and renderer 1427A8650 agree on fade+1E0/1E4;
            // renderer 1427A87C6..8813 separately retains alpha at fade+1F4.
            return fadeBytesMatch(0xEF7E35, std::array<std::uint8_t, 8>{0xF3,0x0F,0x10,0x8F,0x08,0x09,0x00,0x00}) &&
                (native_weapon_fade_suppression::installed() ||
                    fadeBytesMatch(0xEF7EAA, std::array<std::uint8_t, 7>{0x0F,0x2E,0xB7,0x10,0x09,0x00,0x00})) &&
                fadeBytesMatch(0xF4B294, std::array<std::uint8_t, 7>{0x48,0x8B,0x88,0x78,0x01,0x00,0x00}) &&
                fadeBytesMatch(0xF4B2A4, std::array<std::uint8_t, 4>{0x48,0x8B,0x49,0x48}) &&
                fadeBytesMatch(0xF4B2B6, std::array<std::uint8_t, 18>{0x8B,0x45,0x00,0x89,0x81,0xE0,0x01,0x00,0x00,0x8B,0x45,0x00,0x89,0x81,0xE4,0x01,0x00,0x00}) &&
                fadeBytesMatch(0x27ABA3E, std::array<std::uint8_t, 7>{0x48,0x8B,0x88,0x78,0x01,0x00,0x00}) &&
                fadeBytesMatch(0x27A87C6, std::array<std::uint8_t, 20>{0xF3,0x0F,0x11,0xB3,0xE0,0x01,0x00,0x00,0x8B,0x83,0xE0,0x01,0x00,0x00,0x89,0x83,0xF4,0x01,0x00,0x00});
        }

        void recordMeleeFade(const char* phase, RE::NiAVObject* root,
            std::uint64_t transferTrace = 0, const char* role = "equipped") noexcept
        {
            if (!captureThread || !session || !session->meleeFadeReady ||
                !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
            try {
                const float unknown = std::numeric_limits<float>::quiet_NaN();
                auto* player = f4vr::getPlayer();
                std::array<float, 3> combat{unknown, unknown, unknown};
                const bool playerReadable = player && native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const char*>(player) + 0x908, combat.data(), sizeof(combat));
                float disabledAlpha = unknown, lerpSeconds = unknown;
                const bool settingsReadable = native_memory::tryReadValue(
                    reinterpret_cast<const float*>(REL::Offset(0x37CFFA8).address()), disabledAlpha) &&
                    native_memory::tryReadValue(reinterpret_cast<const float*>(REL::Offset(0x37CFFC0).address()), lerpSeconds);
                // Constructor 1427A7D10 starts nodes at zero when this global
                // fade gate is active; 14284E950 enables it after load warm-up.
                std::array<std::uint8_t, 2> renderFadeGates{0xFF, 0xFF};
                std::int32_t renderFrame = -1;
                const bool renderReadable = native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(REL::Offset(0x391AE0C).address()), renderFadeGates.data(), renderFadeGates.size()) &&
                    native_memory::tryReadValue(reinterpret_cast<const std::int32_t*>(REL::Offset(0x391AE28).address()), renderFrame);
                const auto* equipped = f4vr::getEquippedWeaponItem();
                const auto form = equipped && equipped->item.object ? equipped->item.object->formID : 0;
                std::array<void*, 16> owners{};
                std::size_t ownerCount = 0, geometries = 0, unreadable = 0, unbound = 0;
                unsigned deepestFailure = 0;
                void* worstOwner = nullptr;
                float lowestFade = unknown, lowestPrevious = unknown, decal = unknown, multiplier = unknown;
                std::uint64_t flags = 0;
                std::uint8_t category = 0;
                std::int32_t lastRenderFrame = -1;
                bool ownerLimit = false;
                const auto walk = weapon_scene::visitScene(root, [&](RE::NiAVObject* node) {
                    if (!native_memory::pointerRangeLooksReadable(node, sizeof(RE::NiAVObject))) {
                        ++unreadable; deepestFailure = (std::max)(deepestFailure, 1u); return false;
                    }
                    auto* geometry = node->IsGeometry();
                    if (!geometry) return true;
                    ++geometries;
                    void* shader = nullptr;
                    void* owner = nullptr;
                    if (!native_memory::tryReadField(geometry, 0x178, shader)) {
                        ++unreadable; deepestFailure = (std::max)(deepestFailure, 2u); return true;
                    }
                    if (!shader) { ++unbound; return true; }
                    if (!native_memory::tryReadField(shader, 0x48, owner)) {
                        ++unreadable; deepestFailure = (std::max)(deepestFailure, 3u); return true;
                    }
                    std::uintptr_t ownerVtable = 0;
                    if (!owner || !native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(owner), ownerVtable) ||
                        !ownerVtable || !native_memory::pointerRangeLooksReadable(reinterpret_cast<void*>(ownerVtable), sizeof(void*))) {
                        ++unbound; deepestFailure = (std::max)(deepestFailure, 4u); return true;
                    }
                    for (std::size_t i = 0; i < ownerCount; ++i) if (owners[i] == owner) return true;
                    if (ownerCount == owners.size()) { ownerLimit = true; return false; }
                    owners[ownerCount++] = owner;
                    std::array<float, 6> values{};
                    std::uint64_t ownerFlags = 0;
                    float ownerMultiplier = unknown;
                    std::uint8_t ownerCategory = 0;
                    if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const char*>(owner) + 0x1E0,
                            values.data(), sizeof(values)) ||
                        !native_memory::tryReadField(owner, 0x108, ownerFlags) ||
                        !native_memory::tryReadField(owner, 0x118, ownerMultiplier) ||
                        !native_memory::tryReadField(owner, 0x11C, ownerCategory)) {
                        ++unreadable; deepestFailure = (std::max)(deepestFailure, 5u); return true;
                    }
                    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[5]) || !std::isfinite(ownerMultiplier)) {
                        ++unreadable; deepestFailure = (std::max)(deepestFailure, 6u); return true;
                    }
                    if (!std::isfinite(lowestPrevious) || values[5] < lowestPrevious) lowestPrevious = values[5];
                    if (!worstOwner || values[0] < lowestFade) {
                        worstOwner = owner; lowestFade = values[0]; decal = values[1];
                        multiplier = ownerMultiplier; flags = ownerFlags; category = ownerCategory;
                        // Same bounded read above: +1F0 is the renderer's frame
                        // stamp; 1427A86CE uses it for the 20-frame stale reset.
                        std::memcpy(&lastRenderFrame, &values[4], sizeof(lastRenderFrame));
                    }
                    return true;
                });
                session->log->info(
                    "MELEE_FADE phase={} frame={} form={:08X} root=0x{:X} nativeRoot=0x{:X} writes={} requested={} playerReadable={} settingsReadable={} disabledAlpha={} lerp={} cooldown={} cooldownTotal={} cachedAlpha={} geometries={} owners={} worstOwner=0x{:X} currentFade={} decalFade={} multiplier={} minPreviousAlpha={} flags=0x{:X} category={} renderReadable={} renderGates={}/{} renderFrame={} lastRenderFrame={} visited={} truncated={} unreadable={} unbound={} failureStage={} overruns={} transfer={} role={}",
                    phase, runtime_state::currentFrame().frameIndex, form, reinterpret_cast<std::uintptr_t>(root),
                    session->meleeFadeNativeRoot, session->meleeFadeWrites, session->meleeFadeRequested,
                    playerReadable, settingsReadable, disabledAlpha, lerpSeconds, combat[0], combat[1], combat[2],
                    geometries, ownerCount, reinterpret_cast<std::uintptr_t>(worstOwner), lowestFade, decal, multiplier,
                    lowestPrevious, flags, static_cast<unsigned>(category), renderReadable,
                    static_cast<unsigned>(renderFadeGates[0]), static_cast<unsigned>(renderFadeGates[1]), renderFrame, lastRenderFrame,
                    walk.visited, walk.truncated || ownerLimit,
                    unreadable, unbound, deepestFailure, session->pool->overrun_counter(), transferTrace, role);
            } catch (...) {
                ++session->captureFailures;
            }
        }

        bool equipGapWindow(const TransferTrace& trace, std::uint64_t frame) noexcept
        {
            return trace.id && trace.kind == TransferKind::HeldEquip &&
                frame >= trace.startFrame && frame - trace.startFrame < kEquipGapFrames;
        }

        double equipElapsedMs(const TransferTrace& trace) noexcept
        {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - trace.startedAt).count();
        }

        void recordEquipNode(const TransferTrace& trace, const char* phase,
            const char* role, const RE::NiAVObject* model)
        {
            // Ancestors above Weapon matter too. An orphan with AppCulled clear
            // is not evidence of visibility. These are scene observations, not
            // a claim that pixels were submitted by the renderer.
            std::uint32_t culledMask = 0, depth = 0;
            bool reachedWorld = false, reachedSkeleton = false, readable = true;
            const auto* world = f4vr::getWorldRootNode();
            const auto* skeleton = f4vr::getFirstPersonSkeleton();
            const auto* ancestor = model;
            for (; ancestor && depth < 32; ++depth) {
                if (!native_memory::pointerRangeLooksReadable(ancestor, sizeof(RE::NiAVObject))) {
                    readable = false;
                    break;
                }
                if (ancestor->GetAppCulled()) culledMask |= 1u << depth;
                reachedWorld |= ancestor == world;
                reachedSkeleton |= ancestor == skeleton;
                ancestor = ancestor->parent;
            }
            const bool rootReadable = model && native_memory::pointerRangeLooksReadable(model, sizeof(RE::NiAVObject));
            const auto pose = rootReadable ? model->world : RE::NiTransform{};
            session->log->info(
                "EQUIP_NODE trace={} frame={} elapsedMs={:.3f} phase={} role={} model=0x{:X} parent=0x{:X} rootReadable={} ancestryReadable={} depth={} truncated={} culledMask=0x{:X} reachedWorld={} reachedSkeleton={} scale={} xyz=({},{},{}) fadeProbeReady={} overruns={}",
                trace.id, runtime_state::currentFrame().frameIndex, equipElapsedMs(trace), phase, role,
                reinterpret_cast<std::uintptr_t>(model), reinterpret_cast<std::uintptr_t>(rootReadable ? model->parent : nullptr),
                rootReadable, readable, depth, ancestor != nullptr, culledMask, reachedWorld, reachedSkeleton,
                pose.scale, pose.translate.x, pose.translate.y, pose.translate.z,
                session->meleeFadeReady, session->pool->overrun_counter());
            if (rootReadable) recordMeleeFade(phase, const_cast<RE::NiAVObject*>(model), trace.id, role);
        }

        std::uint32_t traceNativeFadeWrite(RE::NiAVObject* root, const float* alpha)
        {
            // Only this verified player presentation call is hooked. Recursion
            // stays native. TLS rejects other threads before session/config access.
            bool sample = false;
            if (captureThread && session && session->meleeFadeReady && g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
                ++session->meleeFadeWrites;
                session->meleeFadeNativeRoot = reinterpret_cast<std::uintptr_t>(root);
                session->meleeFadeRequested = std::numeric_limits<float>::quiet_NaN();
                (void)native_memory::tryReadValue(alpha, session->meleeFadeRequested);
                const auto now = GetTickCount64();
                sample = now >= session->meleeFadeNextSample[0];
                if (sample) {
                    session->meleeFadeNextSample[0] = now + 100;
                    recordMeleeFade("native-before", root);
                }
            }
            const auto result = originalFadeWrite(root, alpha);
            if (sample) recordMeleeFade("native-after", root);
            return result;
        }

        bool installMeleeFadeTrace()
        {
            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
                !validateMeleeFadeLayout()) return false;
            if (originalFadeWrite) return true;
            if (!fadeBytesMatch(0xEF7EE4, std::array<std::uint8_t, 5>{0xE8,0x87,0x33,0x05,0x00})) return false;
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < 32) return false;
            originalFadeWrite = reinterpret_cast<NativeFadeWrite>(REL::Offset(0xF4B270).address());
            trampoline.write_call<5>(REL::Offset(0xEF7EE4).address(), &traceNativeFadeWrite);
            return true;
        }

        native_scope_shot_policy::Ray scopeAxis(const RE::NiTransform& world, unsigned axis) noexcept
        {
            const auto& row = world.rotate.entry[axis];
            return native_scope_shot_policy::ray(
                { world.translate.x, world.translate.y, world.translate.z }, { row[0], row[1], row[2] });
        }

        float scopeRollDegrees(const RE::NiTransform& weapon, const RE::NiTransform& camera) noexcept
        {
            const auto local = transform_math::multiplyStoredRotations(
                camera.rotate, transform_math::transposeRotation(weapon.rotate));
            return native_scope_camera_follow_math::weaponLocalCameraRollDegrees(local);
        }

        void recordFinalScopeDirection(std::uint32_t formId) noexcept
        {
            const auto frame = runtime_state::currentFrame().frameIndex;
            if (session->scopeFrame == 0 || session->scopeFrame != frame) return;
            session->scopeFrame = 0;
            try {
                const auto* weapon = f4vr::getWeaponNode();
                const auto* nodes = f4vr::getPlayerNodes();
                const auto* camera = nodes ? nodes->primaryWeaponScopeCamera : nullptr;
                const bool sameIdentity = weapon && camera && formId == session->scopeFormId &&
                    reinterpret_cast<std::uintptr_t>(weapon) == session->scopeWeaponIdentity &&
                    reinterpret_cast<std::uintptr_t>(camera) == session->scopeCameraIdentity;
                if (!sameIdentity) {
                    session->log->info("SCOPE_DIRECTION phase=after-world-final frame={} generation={:016X} identityChanged=true form={:08X}->{:08X}",
                        frame, session->scopeGeneration, session->scopeFormId, formId);
                    return;
                }
                const auto expected = transform_math::composeTransforms(weapon->world, session->scopeCameraWeaponLocal);
                const auto composed = camera->parent ?
                    transform_math::composeTransforms(camera->parent->world, camera->local) : camera->local;
                const auto weaponRay = scopeAxis(weapon->world, 1);
                const auto cameraRay = scopeAxis(camera->world, 0);
                const auto targetRay = scopeAxis(expected, 0);
                const auto composedRay = scopeAxis(composed, 0);
                session->log->info("SCOPE_DIRECTION phase=after-world-final frame={} generation={:016X} form={:08X} menuOpen={} cameraWeaponDeg={:.4f} targetWeaponDeg={:.4f} cameraTargetDeg={:.4f} worldComposedDeg={:.4f} cameraRollDeg={:.4f} targetRollDeg={:.4f} composedRollDeg={:.4f} weaponForward=({:.5f},{:.5f},{:.5f}) cameraForward=({:.5f},{:.5f},{:.5f}) overruns={} captureFailures={}",
                    frame, session->scopeGeneration, formId, runtime_state::currentFrame().localScopeMenuOpen,
                    native_scope_shot_policy::angleDegrees(cameraRay, weaponRay),
                    native_scope_shot_policy::angleDegrees(targetRay, weaponRay),
                    native_scope_shot_policy::angleDegrees(cameraRay, targetRay),
                    native_scope_shot_policy::angleDegrees(cameraRay, composedRay),
                    scopeRollDegrees(weapon->world, camera->world), scopeRollDegrees(weapon->world, expected),
                    scopeRollDegrees(weapon->world, composed),
                    weaponRay.direction.x, weaponRay.direction.y, weaponRay.direction.z,
                    cameraRay.direction.x, cameraRay.direction.y, cameraRay.direction.z,
                    session->pool->overrun_counter(), session->captureFailures);
            } catch (...) {
                ++session->captureFailures;
            }
        }

        bool targeted(std::uint32_t formId)
        {
            // Exact form IDs bound the investigation; they do not establish
            // that the active model or animation assets are unmodified vanilla.
            return formId == 0x00004822 || formId == 0x0015B043 || formId == 0x00024F55 ||
                   formId == 0x0014831A || formId == 0x0014831B || formId == 0x000DF42E || formId == 0x00171B2B;
        }

        const char* phaseName(Phase phase)
        {
            switch (phase) {
            case Phase::BeforeRockPreFrik: return "before-rock-pre-frik";
            case Phase::BeforeFrik: return "before-frik";
            case Phase::AfterFrik: return "after-frik";
            case Phase::AfterWeaponSolve: return "after-weapon-solve";
            case Phase::AfterRock: return "after-rock";
            case Phase::AfterWorldFinal: return "after-world-final";
            }
            return "unknown";
        }

        bool sampling()
        {
            return session && session->sampling && g_rockConfig.rockDebugWeaponOmodDumpEnabled;
        }

        void recordScopeHandEdge(Phase phase, std::uint64_t frame)
        {
            if (frame == 0 || (phase != Phase::AfterFrik && phase != Phase::AfterRock && phase != Phase::AfterWorldFinal)) return;
            const bool open = frik_visual_authority::isLookingThroughScope();
            if (session->scopeHandStateKnown && session->scopeHandOpen != open) {
                session->scopeHandEdgeFrame = frame;
            }
            session->scopeHandStateKnown = true;
            session->scopeHandOpen = open;
            if (session->scopeHandEdgeFrame == 0 || frame < session->scopeHandEdgeFrame || frame - session->scopeHandEdgeFrame >= 4) return;
            namespace registry = hand_world_claim_registry_policy;
            for (const bool isLeft : { false, true }) {
                const auto hand = frik_visual_authority::handFromBool(isLeft);
                RE::NiTransform tracked{}, input{}, claim{}, rendered{}, solved{};
                frik_visual_authority::ArmChainTransforms chain{};
                const bool trackedValid = frik_visual_authority::tryGetTrackedHandTransform(
                    hand, frik_visual_authority::TrackedHandKind::FirstPersonHand, tracked);
                const bool inputValid = frik_hand_world_authority::tryGetRawHandWorld(isLeft, input);
                const bool claimValid = frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, claim);
                const bool renderedValid = frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, rendered);
                const bool wristValid = frik_visual_authority::tryGetArmChain(hand, chain) && (chain.validMask & (1u << 6)) != 0;
                const auto solve = frik_visual_authority::getHandSolveResult(hand, solved);
                session->log->info(
                    "SCOPE_HAND_EDGE frame={} phase={} age={} open={} hand={} recovery={} tracked={} input={} claim={} wrist={} rendered={} solve={} inputT=({:.3f},{:.3f},{:.3f}) claimT=({:.3f},{:.3f},{:.3f}) wristT=({:.3f},{:.3f},{:.3f}) renderedT=({:.3f},{:.3f},{:.3f}) inputNative={:.3f}gu/{:.3f}deg wristClaim={:.3f}gu/{:.3f}deg",
                    frame, phaseName(phase), frame - session->scopeHandEdgeFrame, open, isLeft ? "left" : "right",
                    frik_hand_world_authority::scopeInputRecoveryMask(), trackedValid, inputValid, claimValid, wristValid, renderedValid,
                    static_cast<unsigned>(solve), input.translate.x, input.translate.y, input.translate.z,
                    claim.translate.x, claim.translate.y, claim.translate.z, chain.hand.translate.x, chain.hand.translate.y,
                    chain.hand.translate.z, rendered.translate.x, rendered.translate.y, rendered.translate.z,
                    inputValid && trackedValid ? registry::translationDeltaGameUnits(input, tracked) : -1.0f,
                    inputValid && trackedValid ? registry::rotationDeltaDegrees(input, tracked) : -1.0f,
                    wristValid && claimValid ? registry::translationDeltaGameUnits(chain.hand, claim) : -1.0f,
                    wristValid && claimValid ? registry::rotationDeltaDegrees(chain.hand, claim) : -1.0f);
            }
        }

        void transform(const char* phase, std::string_view label, const RE::NiTransform& value)
        {
            bool finite = std::isfinite(value.scale) && std::isfinite(value.translate.x) &&
                          std::isfinite(value.translate.y) && std::isfinite(value.translate.z);
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    finite = finite && std::isfinite(value.rotate.entry[row][column]);
                }
            }
            const auto& r = value.rotate.entry;
            session->log->info("VWA transform seq={} phase={} label={} finite={} T=({:.5f},{:.5f},{:.5f}) S={:.6f} R=({:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f})",
                session->sequence, phase, label, finite,
                value.translate.x, value.translate.y, value.translate.z, value.scale,
                r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]);
        }

        std::string_view nodeName(const RE::NiAVObject* node)
        {
            const char* name = node ? node->name.c_str() : nullptr;
            return name ? std::string_view(name).substr(0, 80) : "missing";
        }

        void node(const char* phase, std::string_view role, const RE::NiAVObject* value)
        {
            session->log->info("VWA node seq={} phase={} role={} ptr={:X} name='{}' parent={:X} parentName='{}' flags={:X} parentLocalS={:.6f} parentWorldS={:.6f}",
                session->sequence, phase, role, reinterpret_cast<std::uintptr_t>(value), nodeName(value),
                reinterpret_cast<std::uintptr_t>(value ? value->parent : nullptr),
                nodeName(value ? value->parent : nullptr), value ? value->GetFlags() : 0,
                value && value->parent ? value->parent->local.scale : 0.0f,
                value && value->parent ? value->parent->world.scale : 0.0f);
            if (value) {
                transform(phase, "local", value->local);
                transform(phase, "world", value->world);
            }
        }

        bool selected(const RE::NiAVObject* value)
        {
            const auto name = nodeName(value);
            constexpr std::array names{
                "Weapon", "WeaponOffset", "P-Receiver", "TGunReceiver", "PipeRifleReceiver", "RevolverReceiver",
                "10mmReceiverParentObject", "Pistol10mmReceiver",
                "P-Grip", "P-Barrel", "P-Mag", "P-Scope", "ProjectileNode",
                "WeaponMagazine", "WeaponMagazineTrans", "WeaponTrigger", "WeaponTriggerTrans",
                "WeaponBolt", "WeaponBoltTrans", "WeaponOptics1", "WeaponOptics1Trans",
                "RArm_Hand", "LArm_Hand"
            };
            for (const auto* candidate : names) {
                if (name == candidate) {
                    return true;
                }
            }
            return name.starts_with("Weapon  (") || nodeName(value->parent) == "P-Grip";
        }
    }

    void initialize()
    {
        if (session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
            return;
        }
        try {
            auto next = std::make_unique<Session>();
            const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_WeaponAlignment.log");
            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 8 * 1024 * 1024, 3, true);
            next->pool = std::make_shared<spdlog::details::thread_pool>(1024, 1);
            next->log = std::make_shared<spdlog::async_logger>("ROCK_WeaponAlignment", sink,
                next->pool, spdlog::async_overflow_policy::overrun_oldest);
            next->log->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->log->info("VWA start version=12 authoredSources=unknown:0,live:1,persisted:2,preharvest:3 pid={} build={} {} forms=00004822,0015B043,00024F55,0014831A,0014831B,000DF42E,00171B2B intervalMs=2000 minBoundaryMs=250 matrices=Ni-stored-rows frames=before-rock-pre-frik,before-frik,after-frik,after-rock nativeMask=graph-entry:1,graph-exit:2,primary-entry:4,primary-exit:8,support-entry:16,support-exit:32 nativeThread=game-only looseGrabMinMs=250 sceneMask=weapon:1,receiver:2,muzzle:4,rightHand:8,leftHand:16 aimWritesPerIdentity=48 aimJumpDegrees=5 aimJumpMinMs=250 cycleStride=8 cycleStages=after-frik,after-weapon-solve,after-rock,after-world-final scopeDirectionMinMs=250 scopeAxes=camera-X,weapon-Y invalidAngle=-1 transferFrames=16 transferTerminalFrames=3 transferIK=owner,solve,arm,rendered",
                GetCurrentProcessId(), __DATE__, __TIME__);
            next->log->flush();
            session = std::move(next);
            captureThread = true;
            session->log->info("EQUIP_GAP start version=1 frames={} boundaryClock=steady-ms finalPhase=after-world-final ancestryBudget=32 sceneEvidenceOnly=true poseDeltas=model-origins gate=bDebugWeaponOmodDump",
                kEquipGapFrames);
            session->meleeFadeReady = installMeleeFadeTrace();
            session->log->info("WEAPON_FADE forcedOpaque={} source=native-player-update",
                native_weapon_fade_suppression::installed());
            session->log->info("MELEE_FADE start ready={} nativeMinMs=100 presentedMinMs=100 nodeBudget=512 ownerBudget=16 failureStages=node:1,geometryShader:2,shaderOwner:3,ownerVtable:4,ownerFields:5,nonfinite:6",
                session->meleeFadeReady);
            if (!session->meleeFadeReady) logger::error("ROCK: Native melee fade trace rejected runtime bytes or trampoline capacity; fade sampling disabled.");
            logger::info("ROCK: Vanilla weapon alignment telemetry enabled at '{}'.", path);
        } catch (const std::exception& error) {
            logger::error("ROCK: Vanilla weapon alignment telemetry could not initialize: {}", error.what());
        }
    }

    void shutdown()
    {
        captureThread = false;
        if (session) {
            session->log->info("VWA end overruns={} captureFailures={}", session->pool->overrun_counter(), session->captureFailures);
            session->log->flush();
            session.reset();
        }
    }

    void capture(Phase phase, std::uint64_t schedulerSequence)
    {
        if (!session) {
            return;
        }
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
            session->sampling = false;
            return;
        }
        recordScopeHandEdge(phase, schedulerSequence);
        auto* equipped = f4vr::getEquippedWeaponItem();
        const std::uint32_t formId = equipped && equipped->item.object ? equipped->item.object->formID : 0;
        if (phase == Phase::BeforeRockPreFrik) session->scopeFrame = 0;
        if (phase == Phase::AfterWorldFinal) recordFinalScopeDirection(formId);
        if (phase == Phase::AfterWorldFinal && session->meleeFadeReady) {
            const auto now = GetTickCount64();
            if (now >= session->meleeFadeNextSample[1]) {
                session->meleeFadeNextSample[1] = now + 100;
                recordMeleeFade("presented", f4vr::getWeaponNode());
            }
        }
        // Investigation owner: ROCK/PAPER cycle integration. Compare the
        // solver output, PAPER's publication, and FRIK's later world final.
        // Authority selects the weapon, so a changed mod load index cannot
        // silently disable the trace. Remove after the cycle is qualified.
        const auto authorityFlags = provider::currentNativeAnimationAuthorityFlagsV1();
        using Authority = provider::RockProviderNativeAnimationAuthorityFlagV1;
        const bool handsOnly = (authorityFlags & static_cast<std::uint32_t>(Authority::Hands)) != 0 &&
            (authorityFlags & static_cast<std::uint32_t>(Authority::Weapon)) == 0;
        const bool cyclePhase = phase == Phase::AfterFrik || phase == Phase::AfterWeaponSolve ||
            phase == Phase::AfterRock || phase == Phase::AfterWorldFinal;
        if (cyclePhase && schedulerSequence != 0 && schedulerSequence % 8 == 0 &&
            formId != 0 && (handsOnly || session->cycleTraceSequence == schedulerSequence)) {
            session->cycleTraceSequence = schedulerSequence;
            auto* weapon = f4vr::getWeaponNode();
            const auto* nodes = f4vr::getPlayerNodes();
            const auto* phaseLabel = phaseName(phase);
            session->log->info("CYCLE_TRACE phase={} frame={} scheduler={} form={:08X} authority=0x{:X} weaponValid={} parent={} overruns={}",
                phaseLabel, runtime_state::currentFrame().frameIndex, schedulerSequence, formId, authorityFlags,
                weapon != nullptr, weapon && weapon->parent ? weapon->parent->name.c_str() : "none",
                session->pool->overrun_counter());
            const auto pose = [&](const char* label, const RE::NiTransform& value) {
                const auto& t = value.translate;
                const auto& r = value.rotate.entry;
                session->log->info("CYCLE_TRACE pose phase={} scheduler={} label={} T=({:.5f},{:.5f},{:.5f}) S={:.6f} R=({:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f})",
                    phaseLabel, schedulerSequence, label, t.x, t.y, t.z, value.scale,
                    r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]);
            };
            if (weapon) {
                pose("weapon-world", weapon->world);
                pose("weapon-local", weapon->local);
                if (weapon->parent) pose("weapon-parent", weapon->parent->world);
            }
            if (nodes && nodes->primaryWandNode) pose("right-wand", nodes->primaryWandNode->world);
            if (nodes && nodes->SecondaryWandNode) pose("left-wand", nodes->SecondaryWandNode->world);
            for (const bool isLeft : { false, true }) {
                RE::NiTransform value{};
                if (frik_hand_world_authority::tryGetInputDriverWorld(isLeft, value))
                    pose(isLeft ? "left-driver" : "right-driver", value);
                if (frik_hand_world_authority::tryGetRawHandWorld(isLeft, value))
                    pose(isLeft ? "left-input" : "right-input", value);
                if (frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, value))
                    pose(isLeft ? "left-presented" : "right-presented", value);
            }
        }
        if (phase == Phase::AfterWeaponSolve || phase == Phase::AfterWorldFinal) return;
        if (phase == Phase::BeforeRockPreFrik) {
            session->sequence = schedulerSequence;
            session->nativeMask = 0;
            session->sampling = false;
            if (!targeted(formId) || schedulerSequence == 0) {
                session->formId = 0;
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            const auto interval = formId != session->formId ? std::chrono::milliseconds(250) : std::chrono::milliseconds(2000);
            if (now - session->lastSample < interval) {
                return;
            }
            session->lastSample = now;
            session->sequence = schedulerSequence;
            session->formId = formId;
            session->sampling = true;
        }
        if (!sampling() || session->sequence != schedulerSequence) {
            return;
        }
        const auto* phaseLabel = phaseName(phase);
        if (formId != session->formId) {
            session->log->info("VWA boundary seq={} phase={} form={:08X}->{:08X} sampleAborted=true",
                schedulerSequence, phaseLabel, session->formId, formId);
            session->sampling = false;
            return;
        }
        session->log->info("VWA phase seq={} phase={} form={:08X} overruns={}",
            schedulerSequence, phaseLabel, formId, session->pool->overrun_counter());
        if (phase == Phase::BeforeFrik || phase == Phase::AfterFrik) {
            transform(phaseLabel, "rock-weapon-in-controller", weapon_aim_basis::weaponInController());
        }
        auto* nodes = f4vr::getPlayerNodes();
        node(phaseLabel, "right-wand", nodes ? nodes->primaryWandNode : nullptr);
        node(phaseLabel, "left-wand", nodes ? nodes->SecondaryWandNode : nullptr);
        node(phaseLabel, "right-driver", nodes ? nodes->primaryWeaponOffsetNOde : nullptr);
        node(phaseLabel, "right-recoil", nodes ? nodes->primaryWeaponKickbackRecoilNode : nullptr);

        // All pointers are borrowed for this callback only. Both traversal and
        // output are bounded, even with an unexpected replacement scene graph.
        std::size_t emitted = 0;
        unsigned int sceneMask = 0;
        auto* root = f4vr::getFirstPersonSkeleton();
        node(phaseLabel, "animation-skeleton-root", root);
        // The first-person animation hands are input to the authored capture.
        // The visible body hand is a different node, driven by the root bone
        // tree. Comparing only the former to a body-hand claim invents a pose
        // error even when the rendered hand correctly follows that claim.
        auto* bodyRoot = f4vr::getRootNode();
        node(phaseLabel, "rendered-body-root", bodyRoot);
        unsigned bodyHandMask = 0;
        std::size_t bodyEmitted = 0;
        const auto bodyTraversal = weapon_scene::visitScene(static_cast<RE::NiAVObject*>(bodyRoot), [&](RE::NiAVObject* current) {
            const auto name = nodeName(current);
            const bool right = name == "RArm_Hand";
            const bool left = name == "LArm_Hand";
            const bool finger = name.starts_with("RArm_Finger");
            if (right || left || finger) {
                if (bodyEmitted == 32) return false;
                ++bodyEmitted;
                node(phaseLabel, right ? "rendered-right-hand" : left ? "rendered-left-hand" : "rendered-right-finger", current);
                if (right) bodyHandMask |= 1;
                if (left) bodyHandMask |= 2;
            }
            return true;
        });
        session->log->info("VWA body-end seq={} phase={} visited={} emitted={} truncated={} handMask={:X} sameAsAnimationRoot={}",
            schedulerSequence, phaseLabel, bodyTraversal.visited, bodyEmitted, bodyTraversal.truncated, bodyHandMask, bodyRoot == root);
        if (phase == Phase::AfterRock) {
            for (const bool isLeft : {false, true}) {
                RE::NiTransform claimed{}, presented{};
                const bool hasClaim = frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, claimed);
                const bool hasPresented = frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, presented);
                session->log->info("VWA hand-authority seq={} hand={} claimed={} presented={}",
                    schedulerSequence, isLeft ? "left" : "right", hasClaim, hasPresented);
                if (hasClaim) transform(phaseLabel, isLeft ? "left-winning-claim" : "right-winning-claim", claimed);
                if (hasPresented) transform(phaseLabel, isLeft ? "left-presented-frame" : "right-presented-frame", presented);
            }
        }
        const auto traversal = weapon_scene::visitScene(static_cast<RE::NiAVObject*>(root), [&](RE::NiAVObject* current) {
            if (selected(current)) {
                if (emitted == 40) {
                    return false;
                }
                node(phaseLabel, "scene", current);
                ++emitted;
                const auto name = nodeName(current);
                if (name == "Weapon") sceneMask |= 1;
                if (name == "TGunReceiver" || name == "PipeRifleReceiver" || name == "RevolverReceiver" ||
                    name == "Pistol10mmReceiver" || name == "10mmReceiverParentObject") sceneMask |= 2;
                if (name == "ProjectileNode") sceneMask |= 4;
                if (name == "RArm_Hand") sceneMask |= 8;
                if (name == "LArm_Hand") sceneMask |= 16;
            }
            return true;
        });
        session->log->info("VWA phase-end seq={} phase={} root={:X} visited={} emitted={} truncated={} sceneMask={:X} weaponSceneComplete={} nativeMask={:X} captureFailures={}",
            schedulerSequence, phaseLabel, reinterpret_cast<std::uintptr_t>(root), traversal.visited, emitted,
            traversal.truncated, sceneMask, !traversal.truncated && (sceneMask & 7) == 7,
            session->nativeMask, session->captureFailures);
        if ((sceneMask & 7) != 7 || traversal.truncated) {
            session->log->warn("VWA incomplete-scene seq={} phase={} sceneMask={:X}; missing weapon/receiver/muzzle or traversal bound reached, do not infer transform ownership from this phase",
                schedulerSequence, phaseLabel, sceneMask);
        }
        if (phase == Phase::AfterRock) {
            session->sampling = false;
            session->log->flush();
        }
    }

    void recordNative(NativePhase phase, const RE::NiAVObject* weapon,
        const RE::NiAVObject* offset) noexcept
    {
        if (!captureThread || !sampling()) return;
        const unsigned int bit = 1u << static_cast<unsigned int>(phase);
        if ((session->nativeMask & bit) != 0) return;
        session->nativeMask |= bit;
        try {
            constexpr std::array labels{
                "graph-entry", "graph-exit", "primary-arm-entry", "primary-arm-exit",
                "support-arm-entry", "support-arm-exit"
            };
            const auto* label = labels[static_cast<std::size_t>(phase)];
            auto* equipped = f4vr::getEquippedWeaponItem();
            const auto formId = equipped && equipped->item.object ? equipped->item.object->formID : 0;
            if (formId != session->formId) {
                session->log->info("VWA native-boundary seq={} phase={} form={:08X}->{:08X}",
                    session->sequence, label, session->formId, formId);
                return;
            }
            session->log->info("VWA native seq={} phase={} form={:08X}", session->sequence, label, formId);
            node(label, "native-weapon-argument", weapon);
            node(label, "native-offset-argument", offset);
            auto* root = f4vr::getFirstPersonSkeleton();
            node(label, "skeleton-root", root);
            unsigned int found = 0;
            const auto traversal = weapon_scene::visitScene(static_cast<RE::NiAVObject*>(root), [&](RE::NiAVObject* current) {
                const auto name = nodeName(current);
                if (name == "Weapon") { node(label, "weapon", current); found |= 1; }
                if (name == "RArm_Hand") { node(label, "right-hand", current); found |= 2; }
                if (name == "LArm_Hand") { node(label, "left-hand", current); found |= 4; }
                return true;
            });
            session->log->info("VWA native-end seq={} phase={} mask={:X} visited={} truncated={}",
                session->sequence, label, found, traversal.visited, traversal.truncated);
        } catch (...) {
            // Report the failure count in the enclosing phase/end record;
            // diagnostic allocation/I/O failure cannot cross a native hook.
            ++session->captureFailures;
        }
    }

    void recordNativeAimCapture(const NativeAimCapture& capture) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        // Diagnose the equip/handoff overwrite, including modded weapons.
        // Stable carry stops after this burst; later large changes remain
        // observable at a capped rate. Remove after runtime validation of the aim-source repair.
        if (capture.identityChanged) session->aimCapturesRemaining = 48;
        const float deltaDegrees = capture.previousValid ?
            hand_world_claim_registry_policy::rotationDeltaDegrees(capture.previousAim, capture.nextAim) : 0.0f;
        const auto inputInWand = transform_math::composeTransforms(
            transform_math::invertTransform(capture.wandWorld), capture.inputWorld);
        const float inputDeltaDegrees = hand_world_claim_registry_policy::rotationDeltaDegrees(inputInWand, capture.nextAim);
        const auto now = std::chrono::steady_clock::now();
        const bool largeJump = (capture.previousValid && deltaDegrees >= 5.0f) ||
            (capture.authoredRefreshed && inputDeltaDegrees >= 5.0f);
        if (session->aimCapturesRemaining == 0 &&
            (!largeJump || now - session->lastAimJump < std::chrono::milliseconds(250))) return;
        if (session->aimCapturesRemaining > 0) --session->aimCapturesRemaining;
        if (largeJump) session->lastAimJump = now;
        try {
            std::string_view caller = capture.caller ? capture.caller : "unknown";
            const auto separator = caller.find_last_of("/\\");
            if (separator != std::string_view::npos) caller.remove_prefix(separator + 1);
            session->log->info("VWA aim-write seq={} frame={} form={:08X} caller={}:{} generation={:016X} ownership={:016X} instance={:016X} identityChanged={} previousValid={} source=rock-weapon-aim inputSource={} intentSource={} gripState={} authoredRefreshed={} writeBlocked={} deltaDegrees={:.5f} inputDeltaDegrees={:.5f} burstRemaining={}",
                session->sequence, runtime_state::currentFrame().frameIndex, capture.weaponFormId,
                caller, capture.callerLine, capture.generation, capture.ownership, capture.instanceContent,
                capture.identityChanged, capture.previousValid,
                capture.cleanIntent ? "frame-intent" : "scene-world", capture.intentSource,
                capture.gripState, capture.authoredRefreshed, capture.writeBlocked, deltaDegrees, inputDeltaDegrees,
                session->aimCapturesRemaining);
            if (capture.previousValid) transform("aim-write", "previous-weapon-in-right-wand", capture.previousAim);
            transform("aim-write", "next-weapon-in-right-wand", capture.nextAim);
            transform("aim-write", "input-weapon-world", capture.inputWorld);
            transform("aim-write", "right-wand-world", capture.wandWorld);
            node("aim-write", "weapon-at-capture", capture.weapon);
            if (capture.weapon && capture.weapon->parent) {
                transform("aim-write", "weapon-parent-world", capture.weapon->parent->world);
            }
        } catch (...) {
            ++session->captureFailures;
        }
    }

    void recordScopeCalibration(const RE::NiAVObject* weapon, const RE::NiAVObject* camera,
        std::uint64_t generation, std::uint32_t formId, const RE::NiTransform& nativeCameraWorld,
        const RE::NiTransform& cameraWeaponLocal, bool newlyCaptured) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled || !weapon || !camera) return;
        if (!newlyCaptured && !runtime_state::currentFrame().localScopeMenuOpen) return;
        const auto now = std::chrono::steady_clock::now();
        if (now - session->lastScopeSample < std::chrono::milliseconds(250)) return;
        session->lastScopeSample = now;
        session->scopeFrame = runtime_state::currentFrame().frameIndex;
        session->scopeGeneration = generation;
        session->scopeFormId = formId;
        session->scopeWeaponIdentity = reinterpret_cast<std::uintptr_t>(weapon);
        session->scopeCameraIdentity = reinterpret_cast<std::uintptr_t>(camera);
        session->scopeCameraWeaponLocal = cameraWeaponLocal;
        try {
            const auto target = transform_math::composeTransforms(weapon->world, cameraWeaponLocal);
            const auto weaponRay = scopeAxis(weapon->world, 1);
            const auto nativeRay = scopeAxis(nativeCameraWorld, 0);
            const auto targetRay = scopeAxis(target, 0);
            session->log->info("SCOPE_DIRECTION phase=after-weapon-position frame={} generation={:016X} form={:08X} captured={} menuOpen={} nativeWeaponDeg={:.4f} targetWeaponDeg={:.4f} nativeTargetDeg={:.4f} nativeRollDeg={:.4f} targetRollDeg={:.4f} weaponForward=({:.5f},{:.5f},{:.5f}) nativeForward=({:.5f},{:.5f},{:.5f}) targetForward=({:.5f},{:.5f},{:.5f})",
                session->scopeFrame, generation, formId, newlyCaptured, runtime_state::currentFrame().localScopeMenuOpen,
                native_scope_shot_policy::angleDegrees(nativeRay, weaponRay),
                native_scope_shot_policy::angleDegrees(targetRay, weaponRay),
                native_scope_shot_policy::angleDegrees(nativeRay, targetRay),
                scopeRollDegrees(weapon->world, nativeCameraWorld), scopeRollDegrees(weapon->world, target),
                weaponRay.direction.x, weaponRay.direction.y, weaponRay.direction.z,
                nativeRay.direction.x, nativeRay.direction.y, nativeRay.direction.z,
                targetRay.direction.x, targetRay.direction.y, targetRay.direction.z);
        } catch (...) {
            ++session->captureFailures;
        }
    }

    void recordLooseGrab(RE::TESObjectREFR* ref, bool isLeft, std::uint64_t grabIdentity,
        const RE::NiTransform& handWorld) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled || !ref) return;
        const auto* base = ref->GetObjectReference();
        if (!base || !base->As<RE::TESObjectWEAP>()) return;
        auto& last = session->lastLooseSample[isLeft ? 1u : 0u];
        const auto now = std::chrono::steady_clock::now();
        if (now - last < std::chrono::milliseconds(250)) return;
        last = now;
        try {
            session->log->info("VWA loose-grab seq={} form={:08X} ref={:08X} hand={} grab={}",
                session->sequence, base->formID, ref->formID, isLeft ? "left" : "right", grabIdentity);
            node("loose-grab", "model-root", ref->Get3D());
            transform("loose-grab", "tracked-hand-world", handWorld);
            session->log->flush();
        } catch (...) {
            ++session->captureFailures;
        }
    }

    void beginTransferTrace(TransferKind kind, bool isLeft, std::uint32_t sourceId,
        const RE::NiAVObject* source) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        try {
            const auto frame = runtime_state::currentFrame().frameIndex;
            auto& trace = session->transfers[isLeft ? 1u : 0u];
            trace = { .id = ++session->nextTransferTrace, .startFrame = frame, .kind = kind,
                .sourceId = sourceId, .startedAt = std::chrono::steady_clock::now() };
            if (kind == TransferKind::HeldEquip) {
                const auto* equipped = f4vr::getEquippedWeaponItem();
                const auto form = equipped && equipped->item.object ? equipped->item.object->formID : 0;
                trace.previousNativeModel = reinterpret_cast<std::uintptr_t>(
                    equipped_weapon_visual_state::observe(form).exactInstance);
            }
            const auto& previous = session->lastTransferSource[isLeft ? 1u : 0u];
            const bool previousMatches = source && previous.model == reinterpret_cast<std::uintptr_t>(source) && previous.frame < frame;
            session->log->info("TRANSFER begin trace={} kind={} hand={} source={:08X} frame={} previousFrame={} previousSourceMatches={}",
                trace.id, kind == TransferKind::ToggleDrop ? "toggle-drop" : "held-equip", isLeft ? "left" : "right",
                sourceId, frame, previous.frame, previousMatches);
            if (previousMatches) {
                transform("transfer-previous-final", "source", previous.modelWorld);
                if (previous.handValid) transform("transfer-previous-final", "physical-hand", previous.handWorld);
                if (previous.renderedHandValid) transform("transfer-previous-final", "rendered-hand", previous.renderedHandWorld);
            }
            recordTransferTrace(kind, isLeft, "request", source);
        } catch (...) { ++session->captureFailures; }
    }

    void recordHeldEquipBoundary(std::uint32_t reference, std::uint32_t baseForm,
        const char* stage, const RE::NiAVObject* retainedModel) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        try {
            const auto frame = runtime_state::currentFrame().frameIndex;
            for (auto& trace : session->transfers) {
                if (trace.sourceId != reference || !equipGapWindow(trace, frame)) continue;
                trace.expectedForm = baseForm;
                const auto* equipped = f4vr::getEquippedWeaponItem();
                session->log->info(
                    "EQUIP_BOUNDARY trace={} frame={} elapsedMs={:.3f} phase={} reference={:08X} expected={:08X} equipped={:08X} instance=0x{:X} state={}",
                    trace.id, frame, equipElapsedMs(trace), stage, reference, baseForm,
                    equipped && equipped->item.object ? equipped->item.object->formID : 0,
                    reinterpret_cast<std::uintptr_t>(equipped ? equipped->item.instanceData.get() : nullptr),
                    f4vr::getNativeWeaponState(f4vr::getPlayer()));
                recordEquipNode(trace, stage, "retained-source", retainedModel);
            }
        } catch (...) { ++session->captureFailures; }
    }

    void recordTransferTrace(TransferKind kind, bool isLeft, const char* stage,
        const RE::NiAVObject* model, const RE::NiTransform* target, bool terminal) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        auto& trace = session->transfers[isLeft ? 1u : 0u];
        if (trace.id == 0 || trace.kind != kind) return;
        const auto& runtime = runtime_state::currentFrame();
        const auto frame = runtime.frameIndex;
        if (frame < trace.startFrame) return;
        if (terminal) trace.terminalFrame = frame;
        if (target && model) {
            trace.targetWorld = *target;
            trace.targetModel = reinterpret_cast<std::uintptr_t>(model);
            trace.targetFrame = frame;
        }
        const bool nativeCullEdge = std::string_view(stage) == "native-cull-acquired" ||
            std::string_view(stage) == "native-cull-reasserted" ||
            std::string_view(stage) == "native-cull-restored";
        if (equipGapWindow(trace, frame) &&
            (terminal || nativeCullEdge || std::string_view(stage) == "after-world-final" || std::string_view(stage) == "request")) {
            try {
                recordEquipNode(trace, stage, terminal ? "bridge-release" : nativeCullEdge ? "native-cull" : "bridge-or-source", model);
                if (terminal) {
                    const auto native = equipped_weapon_visual_state::observe(trace.expectedForm, 0, &trace.nativeCache);
                    recordEquipNode(trace, stage, "native-at-release", native.exactInstance);
                } else if (model && std::string_view(stage) == "after-world-final") {
                    trace.bridgeFrame = frame;
                    trace.bridgeModel = reinterpret_cast<std::uintptr_t>(model);
                    trace.bridgeWorld = model->world;
                }
            } catch (...) { ++session->captureFailures; }
        }
        if (!terminal && frame - trace.startFrame >= 16 &&
            (trace.terminalFrame == 0 || frame < trace.terminalFrame || frame - trace.terminalFrame >= 3)) return;
        try {
            RE::NiTransform physical{}, claim{};
            const bool physicalValid = frik_hand_world_authority::tryGetRawHandWorld(isLeft, physical);
            hand_world_claim_registry_policy::Claim claimSnapshot{};
            const bool claimValid = frik_hand_world_authority::tryGetPublishedHandClaim(isLeft, claimSnapshot);
            if (claimValid) claim = claimSnapshot.target;
            auto* nodes = f4vr::getPlayerNodes();
            const auto* wand = nodes ? (isLeft ? nodes->SecondaryWandNode : nodes->primaryWandNode) : nullptr;
            const bool targetMatches = model && trace.targetModel == reinterpret_cast<std::uintptr_t>(model) && trace.targetFrame != 0;
            const auto error = targetMatches ? model->world.translate - trace.targetWorld.translate : RE::NiPoint3{};
            session->log->info("TRANSFER frame trace={} kind={} hand={} source={:08X} frame={} age={} phase={} model={:X} parent={:X} parentName='{}' visible={} physical={} claim={} wand={} targetMatches={} targetFrame={} error=({:.5f},{:.5f},{:.5f}) roomValid={} roomStep=({:.5f},{:.5f},{:.5f}) overruns={}",
                trace.id, kind == TransferKind::ToggleDrop ? "toggle-drop" : "held-equip", isLeft ? "left" : "right",
                trace.sourceId, frame, frame - trace.startFrame, stage, reinterpret_cast<std::uintptr_t>(model),
                reinterpret_cast<std::uintptr_t>(model ? model->parent : nullptr), nodeName(model ? model->parent : nullptr),
                model && !model->GetAppCulled(), physicalValid, claimValid, wand != nullptr, targetMatches, trace.targetFrame,
                error.x, error.y, error.z, runtime.playerSpace.valid,
                runtime.playerSpace.deltaGameUnits.x, runtime.playerSpace.deltaGameUnits.y, runtime.playerSpace.deltaGameUnits.z,
                session->pool->overrun_counter());
            if (model) {
                transform(stage, "model-world", model->world);
                transform(stage, "model-previous-world", model->previousWorld);
            }
            if (physicalValid) transform(stage, "physical-hand", physical);
            if (claimValid) transform(stage, "claimed-hand", claim);
            if (wand) transform(stage, "wand-world", wand->world);
            if (targetMatches) transform(stage, "target-world", trace.targetWorld);
            if (std::string_view(stage) == "request" || std::string_view(stage) == "after-rock" ||
                std::string_view(stage) == "final-hand") {
                frik_visual_authority::ArmChainTransforms arm{};
                const auto hand = frik_visual_authority::handFromBool(isLeft);
                const bool armValid = frik_visual_authority::tryGetArmChain(hand, arm);
                RE::NiTransform solved{}, rendered{};
                const auto solve = frik_visual_authority::getHandSolveResult(hand, solved);
                const bool renderedValid = std::string_view(stage) == "final-hand" &&
                    frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, rendered);
                session->log->info("TRANSFER ik trace={} frame={} hand={} phase={} owner='{}' priority={} fallback={} solve={} armMask={:X} rendered={}",
                    trace.id, frame, isLeft ? "left" : "right", stage,
                    claimValid ? claimSnapshot.tag.data() : "none", claimSnapshot.priority,
                    claimSnapshot.fallbackReported, static_cast<unsigned>(solve), armValid ? arm.validMask : 0, renderedValid);
                if (armValid && (arm.validMask & (1u << 1))) transform(stage, "upper-arm", arm.upperArm);
                if (armValid && (arm.validMask & (1u << 3))) transform(stage, "forearm", arm.forearm1);
                if (armValid && (arm.validMask & (1u << 6))) transform(stage, "hand-node", arm.hand);
                if (renderedValid) transform(stage, "rendered-hand", rendered);
                if (solve != frik_visual_authority::HandSolveState::SkeletonNotReady) transform(stage, "solved-wrist", solved);
            }
        } catch (...) { ++session->captureFailures; }
    }

    void recordTransferGrabGate(bool isLeft, std::uint64_t grabId, const char* phase, const char* reason,
        bool transferred, bool authoredPose, bool offsetActive, bool settled, bool presented, bool publish) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        const auto& trace = session->transfers[isLeft ? 1u : 0u];
        const auto frame = runtime_state::currentFrame().frameIndex;
        if (!trace.id || trace.kind != TransferKind::ToggleDrop || frame < trace.startFrame || frame - trace.startFrame >= 16) return;
        try {
            session->log->info("TRANSFER grab-gate trace={} frame={} hand={} grab={} phase={} gate={} transferred={} authored={} offset={} settled={} presented={} publish={}",
                trace.id, frame, isLeft ? "left" : "right", grabId, phase, reason,
                transferred, authoredPose, offsetActive, settled, presented, publish);
        } catch (...) { ++session->captureFailures; }
    }

    void captureTransferFrame(bool isLeft, const RE::NiAVObject* looseRoot,
        const RE::NiAVObject* equippedRoot) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        const auto index = isLeft ? 1u : 0u;
        auto& trace = session->transfers[index];
        const auto kind = trace.kind;
        const auto frame = runtime_state::currentFrame().frameIndex;
        if (equipGapWindow(trace, frame) && trace.finalFrame != frame) {
            trace.finalFrame = frame;
            try {
                const auto* equipped = f4vr::getEquippedWeaponItem();
                const auto currentForm = equipped && equipped->item.object ? equipped->item.object->formID : 0;
                const auto native = equipped_weapon_visual_state::observe(trace.expectedForm, 0, &trace.nativeCache);
                recordEquipNode(trace, "after-world-final", "native", native.exactInstance);
                if (looseRoot) recordEquipNode(trace, "after-world-final", "held-loose", looseRoot);
                const bool comparable = native.exactInstance && trace.bridgeFrame != 0;
                const auto delta = comparable ? native.exactInstance->world.translate - trace.bridgeWorld.translate : RE::NiPoint3{};
                // Model origins can differ between loose and equipped assets;
                // these deltas are witnesses, not an automatic alignment test.
                session->log->info(
                    "EQUIP_FRAME trace={} frame={} age={} elapsedMs={:.3f} hand={} reference={:08X} expected={:08X} equipped={:08X} instance=0x{:X} state={} heldLoose={} bridgeThisFrame={} bridge=0x{:X} bridgeFrame={} native=0x{:X} sameAsPreviousNative={} ancestorsVisible={} locallyVisible={} comparable={} originDistance={} rotationDegrees={} bridgeScale={} nativeScale={} overruns={}",
                    trace.id, frame, frame - trace.startFrame, equipElapsedMs(trace), isLeft ? "left" : "right",
                    trace.sourceId, trace.expectedForm, currentForm,
                    reinterpret_cast<std::uintptr_t>(equipped ? equipped->item.instanceData.get() : nullptr),
                    f4vr::getNativeWeaponState(f4vr::getPlayer()), looseRoot != nullptr,
                    trace.bridgeFrame == frame, trace.bridgeModel, trace.bridgeFrame,
                    reinterpret_cast<std::uintptr_t>(native.exactInstance),
                    native.exactInstance && reinterpret_cast<std::uintptr_t>(native.exactInstance) == trace.previousNativeModel,
                    native.ancestorPathVisible, native.instanceLocallyVisible, comparable,
                    comparable ? std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) : -1.0f,
                    comparable ? hand_world_claim_registry_policy::rotationDeltaDegrees(trace.bridgeWorld, native.exactInstance->world) : -1.0f,
                    trace.bridgeFrame ? trace.bridgeWorld.scale : 0.0f,
                    native.exactInstance ? native.exactInstance->world.scale : 0.0f, session->pool->overrun_counter());
            } catch (...) { ++session->captureFailures; }
        } else if (trace.id && kind == TransferKind::HeldEquip && !trace.gapWindowEnded &&
            frame >= trace.startFrame && frame - trace.startFrame >= kEquipGapFrames) {
            trace.gapWindowEnded = true;
            try {
                session->log->info("EQUIP_WINDOW_END trace={} frame={} elapsedMs={:.3f} limitFrames={} releaseFrame={} overruns={} failures={}",
                    trace.id, frame, equipElapsedMs(trace), kEquipGapFrames, trace.terminalFrame,
                    session->pool->overrun_counter(), session->captureFailures);
            } catch (...) { ++session->captureFailures; }
        }
        recordTransferTrace(kind, isLeft, "final-hand", nullptr);
        if (looseRoot) recordTransferTrace(kind, isLeft, "final-loose", looseRoot);
        if (equippedRoot) recordTransferTrace(kind, isLeft, "final-equipped", equippedRoot);
        auto& last = session->lastTransferSource[index];
        last = {};
        if (const auto* source = looseRoot ? looseRoot : equippedRoot) {
            last.frame = runtime_state::currentFrame().frameIndex;
            last.model = reinterpret_cast<std::uintptr_t>(source);
            last.modelWorld = source->world;
            last.handValid = frik_hand_world_authority::tryGetRawHandWorld(isLeft, last.handWorld);
            last.renderedHandValid = frik_hand_world_authority::tryGetPresentedHandWorld(isLeft, last.renderedHandWorld);
        }
    }

    void recordTransferPose(std::uint32_t refId, bool isLeft, const char* stage,
        const RE::NiTransform& weaponWorld, const RE::NiTransform& handWorld,
        const RE::NiTransform* proxyWorld, const RE::NiTransform* desiredWeaponWorld) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        try {
            session->log->info("VWA transfer-pose seq={} ref={:08X} hand={} stage={}",
                session->sequence, refId, isLeft ? "left" : "right", stage);
            transform("transfer", "weapon-world", weaponWorld);
            transform("transfer", "physical-hand-world", handWorld);
            if (proxyWorld) transform("transfer", "proxy-world", *proxyWorld);
            if (desiredWeaponWorld) transform("transfer", "desired-weapon-world", *desiredWeaponWorld);
            if (std::isfinite(handWorld.scale) && std::abs(handWorld.scale) > 0.0001f) {
                transform("transfer", "weapon-in-physical-hand", transform_math::composeTransforms(
                    transform_math::invertTransform(handWorld), weaponWorld));
            }
        } catch (...) { ++session->captureFailures; }
    }

    void recordAuthoredPose(std::uint32_t formId, std::uint64_t captureSequence,
        const char* source, const char* label, const RE::NiTransform& pose) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        try {
            session->log->info("VWA authored-pose seq={} form={:08X} capture={} source={} label={}",
                session->sequence, formId, captureSequence, source, label);
            transform("authored-source", label, pose);
        } catch (...) { ++session->captureFailures; }
    }

    void recordAuthoredSelection(const AuthoredPrimaryFiringGripFrameInput& input,
        const authored_weapon_grip_library::WeaponVariantIdentity& requested,
        const authored_weapon_grip_library::LookupResult& selected,
        const RE::NiPoint3& modelTranslation, bool compiledMinigunSeat) noexcept
    {
        if (!captureThread || !sampling() || !input.weapon || input.weapon->formID != session->formId) return;
        try {
            session->log->info("VWA authored-selection seq={} form={:08X} requestedVariant={:016X} instance={:016X} instanceKnown={} found={} source={} capture={} variantFallback={} reason={} support={} supportSource={} supportCapture={} rightMask={:04X} supportMask={:04X} compiledMinigun={} registration=({:.6f},{:.6f},{:.6f})",
                session->sequence, input.weapon->formID, requested.key, requested.instanceContentKey,
                requested.instanceContentKnown, selected.found, static_cast<unsigned>(selected.source),
                selected.captureSequence, selected.usedVariantFallback, selected.reason,
                selected.hasSupportRelation, static_cast<unsigned>(selected.supportSource), selected.supportCaptureSequence,
                selected.rightFiringFingerPose.enabledMask, selected.supportFingerPose.enabledMask,
                compiledMinigunSeat, modelTranslation.x, modelTranslation.y, modelTranslation.z);
            if (selected.found) {
                transform("authored-selection", "raw-right-hand-in-weapon", selected.rightHandWeaponLocal);
                if (selected.hasSupportRelation) transform("authored-selection", "raw-support-hand-in-weapon", selected.supportHandWeaponLocal);
                for (std::size_t i = 0; i < selected.rightFiringFingerPose.localTransforms.size(); ++i) {
                    if ((selected.rightFiringFingerPose.enabledMask & (1u << i)) == 0) continue;
                    session->log->info("VWA authored-finger seq={} capture={} index={}", session->sequence, selected.captureSequence, i);
                    transform("authored-selection", "right-finger-local", selected.rightFiringFingerPose.localTransforms[i]);
                }
            }
            RE::NiTransform liveRelation{};
            std::uint64_t liveSequence = 0;
            const bool liveAvailable = authored_weapon_grip_capture::tryGetPrimaryFiringGripRelation(
                input.weaponNode, liveRelation, liveSequence);
            session->log->info("VWA authored-live-comparison seq={} available={} capture={}", session->sequence, liveAvailable, liveSequence);
            if (liveAvailable) transform("authored-selection", "animation-graph-hand-in-weapon", liveRelation);
        } catch (...) { ++session->captureFailures; }
    }

    void recordInput(const AuthoredPrimaryFiringGripFrameInput& input)
    {
        if (!sampling() || !input.weapon || input.weapon->formID != session->formId) {
            return;
        }
        session->log->info("VWA authored-input seq={} form={:08X} generation={:X} ownership={:X} instance={:X} instanceKnown={} node={:X} pa={} leftFiring={} initialized={} visual={} skeleton={} menu={} compatibility={} drawn={} visible={} reload={} conflict={} return={} transition={} holdingObject={}",
            session->sequence, session->formId, input.weaponGenerationKey, input.weaponOwnershipKey,
            input.weaponInstanceContentKey, input.weaponInstanceContentKnown, reinterpret_cast<std::uintptr_t>(input.weaponNode),
            input.inPowerArmor, input.rockFiringHandIsLeft, input.runtimeInitialized, input.visualAuthorityAvailable,
            input.localSkeletonReady, input.menuBlocking, input.compatibilityBlocking, input.weaponDrawn, input.weaponVisible,
            input.nativeReloadAuthorityActive, input.conflictingWeaponTransformAuthorityActive,
            input.weaponVisualReturnActive, input.equippedWeaponTransitionActive, input.primaryHandHoldingObject);
    }

    void recordSolve(std::uint32_t formId, std::uint64_t captureSequence, const char* source,
        const RE::NiTransform& handInWeapon, const RE::NiTransform& trackedHand, const RE::NiTransform& solvedWeapon)
    {
        if (!sampling() || formId != session->formId) {
            return;
        }
        session->log->info("VWA authored-solve seq={} form={:08X} capture={} source={}",
            session->sequence, formId, captureSequence, source);
        transform("authored-solve", "hand-in-weapon", handInWeapon);
        transform("authored-solve", "tracked-hand-world", trackedHand);
        transform("authored-solve", "proposed-weapon-world", solvedWeapon);
    }
}
