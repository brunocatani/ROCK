#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"

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
            std::uint32_t formId{ 0 };
            bool sampling{ false };
            unsigned int nativeMask{ 0 };
            std::uint64_t captureFailures{ 0 };
            std::array<std::chrono::steady_clock::time_point, 2> lastLooseSample{};
        };
        std::unique_ptr<Session> session;
        // Lifecycle and phase capture share the game thread. Native animation
        // workers see false in their own TLS and never read the session pointer.
        thread_local bool captureThread = false;

        bool targeted(std::uint32_t formId)
        {
            // Exact form IDs bound the investigation; they do not establish
            // that the active model or animation assets are unmodified vanilla.
            return formId == 0x00004822 || formId == 0x0015B043 || formId == 0x00024F55 ||
                   formId == 0x0014831A || formId == 0x0014831B;
        }

        const char* phaseName(Phase phase)
        {
            switch (phase) {
            case Phase::BeforeRockPreFrik: return "before-rock-pre-frik";
            case Phase::BeforeFrik: return "before-frik";
            case Phase::AfterFrik: return "after-frik";
            case Phase::AfterRock: return "after-rock";
            }
            return "unknown";
        }

        bool sampling()
        {
            return session && session->sampling && g_rockConfig.rockDebugWeaponOmodDumpEnabled;
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
            next->log->info("VWA start version=4 authoredSources=unknown:0,live:1,persisted:2,preharvest:3 pid={} build={} {} forms=00004822,0015B043,00024F55,0014831A,0014831B intervalMs=2000 minBoundaryMs=250 matrices=Ni-stored-rows frames=before-rock-pre-frik,before-frik,after-frik,after-rock nativeMask=graph-entry:1,graph-exit:2,primary-entry:4,primary-exit:8,support-entry:16,support-exit:32 nativeThread=game-only looseGrabMinMs=250 sceneMask=weapon:1,receiver:2,muzzle:4,rightHand:8,leftHand:16",
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
        auto* equipped = f4vr::getEquippedWeaponItem();
        const std::uint32_t formId = equipped && equipped->item.object ? equipped->item.object->formID : 0;
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
        node(phaseLabel, "skeleton-root", root);
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

    void recordLooseGrab(RE::TESObjectREFR* ref, bool isLeft, std::uint64_t grabIdentity,
        const RE::NiTransform& handWorld) noexcept
    {
        if (!captureThread || !session || !g_rockConfig.rockDebugWeaponOmodDumpEnabled || !ref) return;
        const auto* base = ref->GetObjectReference();
        if (!base || !targeted(base->formID)) return;
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
            if (liveAvailable) transform("authored-selection", "live-graph-hand-in-weapon", liveRelation);
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
