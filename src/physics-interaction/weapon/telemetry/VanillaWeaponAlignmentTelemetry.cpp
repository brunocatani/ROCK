#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
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
            RE::NiTransform modelWorld{}, handWorld{};
            bool handValid{};
        };
        struct TransferTrace
        {
            std::uint64_t id{}, startFrame{}, terminalFrame{}, targetFrame{};
            TransferKind kind{};
            std::uint32_t sourceId{};
            std::uintptr_t targetModel{};
            RE::NiTransform targetWorld{};
        };
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
            // Scope-direction investigation: remove after the capture-phase
            // repair is qualified. Values/identity tokens only, no held nodes.
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
        };
        std::unique_ptr<Session> session;
        // Lifecycle and phase capture share the game thread. Native animation
        // workers see false in their own TLS and never read the session pointer.
        thread_local bool captureThread = false;

        native_scope_shot_policy::Ray scopeAxis(const RE::NiTransform& world, unsigned axis) noexcept
        {
            const auto& row = world.rotate.entry[axis];
            return native_scope_shot_policy::ray(
                { world.translate.x, world.translate.y, world.translate.z }, { row[0], row[1], row[2] });
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
                session->log->info("SCOPE_DIRECTION phase=after-world-final frame={} generation={:016X} form={:08X} menuOpen={} cameraWeaponDeg={:.4f} targetWeaponDeg={:.4f} cameraTargetDeg={:.4f} worldComposedDeg={:.4f} weaponForward=({:.5f},{:.5f},{:.5f}) cameraForward=({:.5f},{:.5f},{:.5f}) overruns={} captureFailures={}",
                    frame, session->scopeGeneration, formId, runtime_state::currentFrame().localScopeMenuOpen,
                    native_scope_shot_policy::angleDegrees(cameraRay, weaponRay),
                    native_scope_shot_policy::angleDegrees(targetRay, weaponRay),
                    native_scope_shot_policy::angleDegrees(cameraRay, targetRay),
                    native_scope_shot_policy::angleDegrees(cameraRay, composedRay),
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
            next->log->info("VWA start version=11 authoredSources=unknown:0,live:1,persisted:2,preharvest:3 pid={} build={} {} forms=00004822,0015B043,00024F55,0014831A,0014831B,000DF42E,00171B2B intervalMs=2000 minBoundaryMs=250 matrices=Ni-stored-rows frames=before-rock-pre-frik,before-frik,after-frik,after-rock nativeMask=graph-entry:1,graph-exit:2,primary-entry:4,primary-exit:8,support-entry:16,support-exit:32 nativeThread=game-only looseGrabMinMs=250 sceneMask=weapon:1,receiver:2,muzzle:4,rightHand:8,leftHand:16 aimWritesPerIdentity=48 aimJumpDegrees=5 aimJumpMinMs=250 cycleStride=8 cycleStages=after-frik,after-weapon-solve,after-rock,after-world-final scopeDirectionMinMs=250 scopeAxes=camera-X,weapon-Y invalidAngle=-1 transferFrames=8 transferTerminalFrames=3",
                GetCurrentProcessId(), __DATE__, __TIME__);
            next->log->flush();
            session = std::move(next);
            captureThread = true;
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
            session->log->info("VWA aim-write seq={} frame={} form={:08X} caller={}:{} generation={:016X} ownership={:016X} instance={:016X} identityChanged={} previousValid={} source=rock-controller-basis inputSource={} intentSource={} gripState={} authoredRefreshed={} writeBlocked={} deltaDegrees={:.5f} inputDeltaDegrees={:.5f} burstRemaining={}",
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
            session->log->info("SCOPE_DIRECTION phase=after-weapon-position frame={} generation={:016X} form={:08X} captured={} menuOpen={} nativeWeaponDeg={:.4f} targetWeaponDeg={:.4f} nativeTargetDeg={:.4f} weaponForward=({:.5f},{:.5f},{:.5f}) nativeForward=({:.5f},{:.5f},{:.5f}) targetForward=({:.5f},{:.5f},{:.5f})",
                session->scopeFrame, generation, formId, newlyCaptured, runtime_state::currentFrame().localScopeMenuOpen,
                native_scope_shot_policy::angleDegrees(nativeRay, weaponRay),
                native_scope_shot_policy::angleDegrees(targetRay, weaponRay),
                native_scope_shot_policy::angleDegrees(nativeRay, targetRay),
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
            trace = { .id = ++session->nextTransferTrace, .startFrame = frame, .kind = kind, .sourceId = sourceId };
            const auto& previous = session->lastTransferSource[isLeft ? 1u : 0u];
            const bool previousMatches = source && previous.model == reinterpret_cast<std::uintptr_t>(source) && previous.frame < frame;
            session->log->info("TRANSFER begin trace={} kind={} hand={} source={:08X} frame={} previousFrame={} previousSourceMatches={}",
                trace.id, kind == TransferKind::ToggleDrop ? "toggle-drop" : "held-equip", isLeft ? "left" : "right",
                sourceId, frame, previous.frame, previousMatches);
            if (previousMatches) {
                transform("transfer-previous-final", "source", previous.modelWorld);
                if (previous.handValid) transform("transfer-previous-final", "physical-hand", previous.handWorld);
            }
            recordTransferTrace(kind, isLeft, "request", source);
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
        if (!terminal && frame - trace.startFrame >= 8 &&
            (trace.terminalFrame == 0 || frame < trace.terminalFrame || frame - trace.terminalFrame >= 3)) return;
        try {
            RE::NiTransform physical{}, claim{};
            const bool physicalValid = frik_hand_world_authority::tryGetRawHandWorld(isLeft, physical);
            const bool claimValid = frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, claim);
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
        } catch (...) { ++session->captureFailures; }
    }

    void captureTransferFrame(bool isLeft, const RE::NiAVObject* looseRoot,
        const RE::NiAVObject* equippedRoot) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        const auto index = isLeft ? 1u : 0u;
        const auto kind = session->transfers[index].kind;
        if (looseRoot) recordTransferTrace(kind, isLeft, "final-loose", looseRoot);
        if (equippedRoot) recordTransferTrace(kind, isLeft, "final-equipped", equippedRoot);
        auto& last = session->lastTransferSource[index];
        last = {};
        if (const auto* source = looseRoot ? looseRoot : equippedRoot) {
            last.frame = runtime_state::currentFrame().frameIndex;
            last.model = reinterpret_cast<std::uintptr_t>(source);
            last.modelWorld = source->world;
            last.handValid = frik_hand_world_authority::tryGetRawHandWorld(isLeft, last.handWorld);
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
