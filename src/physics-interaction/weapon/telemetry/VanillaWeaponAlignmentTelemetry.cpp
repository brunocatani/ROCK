#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"
#include "physics-interaction/weapon/telemetry/WeaponTelemetryTraversal.h"

#include "RockConfig.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/weapon/AuthoredPrimaryFiringGrip.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"

#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <algorithm>
#include <array>
#include <atomic>
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
        // capture once SMG translation and pipe rotation have been qualified.
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
            bool failed{ false };
            struct SampleGate
            {
                std::chrono::steady_clock::time_point last{};
                std::uint32_t form = 0;
            };
            std::array<SampleGate, 3> armGates{};
            SampleGate animationGate{};
            std::uint64_t event = 0;
        };
        std::unique_ptr<Session> session;
        std::atomic<DWORD> ownerThread{ 0 };

        bool targeted(std::uint32_t formId)
        {
            // Exact runtime IDs of the four Fallout4.esm forms; never match
            // names, low FormID bits, keywords, or modded copies of these guns.
            return formId == 0x0015B043 || formId == 0x00024F55 ||
                   formId == 0x0014831A || formId == 0x0014831B;
        }

        const char* phaseName(Phase phase)
        {
            switch (phase) {
            case Phase::BeforeFrik: return "before-frik";
            case Phase::AfterFrik: return "after-frik";
            case Phase::AfterRock: return "after-rock";
            }
            return "unknown";
        }

        bool sampling()
        {
            return session && !session->failed && session->sampling && g_rockConfig.rockDebugWeaponOmodDumpEnabled;
        }

        bool diagnosticAllowed() noexcept
        {
            return ownerThread.load(std::memory_order_acquire) == GetCurrentThreadId() &&
                   session && !session->failed && g_rockConfig.rockDebugWeaponOmodDumpEnabled;
        }

        bool admit(Session::SampleGate& gate, std::uint32_t form)
        {
            if (!targeted(form)) return false;
            const auto now = std::chrono::steady_clock::now();
            const auto interval = gate.form == form ? std::chrono::milliseconds(2000) : std::chrono::milliseconds(250);
            if (now - gate.last < interval) return false;
            gate = { now, form };
            return true;
        }

        void disableAfterFailure() noexcept
        {
            if (!session || session->failed) return;
            session->failed = true;
            try { logger::error("ROCK: Vanilla alignment capture disabled after a diagnostic failure; data is incomplete."); } catch (...) {}
        }

        void transform(const char* phase, std::string_view label, const RE::NiTransform& value, std::uint64_t event = 0)
        {
            bool finite = std::isfinite(value.scale) && std::isfinite(value.translate.x) &&
                          std::isfinite(value.translate.y) && std::isfinite(value.translate.z);
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column) {
                    finite = finite && std::isfinite(value.rotate.entry[row][column]);
                }
            }
            const auto& r = value.rotate.entry;
            session->log->info("VWA transform seq={} phase={} label={} finite={} T=({:.5f},{:.5f},{:.5f}) S={:.6f} R=({:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f}) event={}",
                session->sequence, phase, label, finite,
                value.translate.x, value.translate.y, value.translate.z, value.scale,
                r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2], event);
        }

        std::string_view nodeName(const RE::NiAVObject* node)
        {
            const char* name = node ? node->name.c_str() : nullptr;
            return name ? std::string_view(name).substr(0, 80) : "missing";
        }

        void node(const char* phase, std::string_view role, const RE::NiAVObject* value, std::uint64_t event = 0)
        {
            session->log->info("VWA node seq={} phase={} role={} ptr={:X} name='{}' parent={:X} parentName='{}' flags={:X} event={}",
                session->sequence, phase, role, reinterpret_cast<std::uintptr_t>(value), nodeName(value),
                reinterpret_cast<std::uintptr_t>(value ? value->parent : nullptr),
                nodeName(value ? value->parent : nullptr), value ? value->GetFlags() : 0, event);
            if (value) {
                transform(phase, "local", value->local, event);
                transform(phase, "world", value->world, event);
            }
        }

        bool selected(const RE::NiAVObject* value)
        {
            const auto name = nodeName(value);
            constexpr std::array names{
                "Weapon", "WeaponOffset", "P-Receiver", "TGunReceiver", "PipeRifleReceiver", "RevolverReceiver",
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

        std::uint32_t equippedForm()
        {
            const auto* equipped = f4vr::getEquippedWeaponItem();
            return equipped && equipped->item.object ? equipped->item.object->formID : 0;
        }

        void armScene(std::uint64_t event, ArmCaller caller, bool before,
            RE::NiNode** weapon, RE::NiNode** offset)
        {
            const auto* phase = before ? "arm-before" : "arm-after";
            RE::NiNode* weaponNode = nullptr;
            RE::NiNode* offsetNode = nullptr;
            const bool weaponRead = native_memory::tryReadValue(weapon, weaponNode);
            const bool offsetRead = native_memory::tryReadValue(offset, offsetNode);
            session->log->info("VWA arm event={} phase={} caller={} form={:08X} weaponArgValid={} offsetArgValid={}",
                event, phase, static_cast<unsigned>(caller), equippedForm(), weaponRead, offsetRead);
            node(phase, "argument-weapon", weaponNode, event);
            node(phase, "argument-offset", offsetNode, event);
            std::size_t emitted = 0;
            const auto traversal = visitScene(static_cast<RE::NiAVObject*>(weaponNode), [&](RE::NiAVObject* current) {
                if (!selected(current)) return true;
                if (emitted == 24) return false;
                node(phase, "weapon-subtree", current, event);
                ++emitted;
                return true;
            });
            session->log->info("VWA arm-end event={} phase={} visited={} emitted={} truncated={}",
                event, phase, traversal.visited, emitted, traversal.truncated);
        }

        bool guardedArmScene(std::uint64_t event, ArmCaller caller, bool before,
            RE::NiNode** weapon, RE::NiNode** offset)
        {
#if defined(_MSC_VER)
            __try {
                armScene(event, caller, before, weapon, offset);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            armScene(event, caller, before, weapon, offset);
            return true;
#endif
        }

        void handPresentation(const char* phase)
        {
            const auto trace = frik_hand_world_authority::scopeDampenTrace();
            // Called only after ROCK's final presentation. The sequence fields
            // distinguish current inputs from missing or retained snapshots.
            session->log->info("VWA hand-authority seq={} driverSeq={} observedSeq={} rawValid={} consumedValid={} claimedValid={} presentedValid={}",
                session->sequence, trace.driverSequence, trace.observedSequence, trace.raw[0].valid,
                trace.consumed[0].valid, trace.claimed[0].valid, trace.presented[0].valid);
            if (trace.raw[0].valid) transform(phase, "raw-controller-driver", trace.raw[0].world);
            if (trace.consumed[0].valid) transform(phase, "frik-consumed-hand", trace.consumed[0].target);
            if (trace.claimed[0].valid) transform(phase, "rock-claimed-hand", trace.claimed[0].target);
            if (trace.presented[0].valid) transform(phase, "cached-presented-hand", trace.presented[0].world);
            auto* tree = f4vr::getFlattenedBoneTree();
            if (!tree || !tree->transforms || tree->numTransforms <= 0 || tree->numTransforms > 768) {
                session->log->warn("VWA body-hand seq={} treeValid=false", session->sequence);
                return;
            }
            for (int i = 0; i < tree->numTransforms; ++i) {
                const auto& bone = tree->transforms[i];
                if (std::string_view(bone.name.c_str() ? bone.name.c_str() : "") != "RArm_Hand") continue;
                session->log->info("VWA body-hand seq={} treeValid=true index={} parent={} ref={:X}",
                    session->sequence, i, bone.parPos, reinterpret_cast<std::uintptr_t>(bone.refNode));
                transform(phase, "body-flattened-hand-world", bone.world);
                node(phase, "body-hand-ref-node", bone.refNode);
                return;
            }
            session->log->warn("VWA body-hand seq={} treeValid=true handFound=false", session->sequence);
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
            next->log->info("VWA start version=3 pid={} build={} {} forms=0015B043,00024F55,0014831A,0014831B intervalMs=2000 minBoundaryMs=250 matrices=Ni-stored-rows frames=before-frik,after-frik,after-rock sceneMask=weapon:1,receiver:2,muzzle:4,rightHand:8,leftHand:16 armCallers=nativePrimary:0,nativeSupport:1,other:2 armEvents=independent-of-frame-sampling",
                GetCurrentProcessId(), __DATE__, __TIME__);
            next->log->flush();
            session = std::move(next);
            ownerThread.store(GetCurrentThreadId(), std::memory_order_release);
            logger::info("ROCK: Vanilla weapon alignment telemetry enabled at '{}'.", path);
        } catch (const std::exception& error) {
            logger::error("ROCK: Vanilla weapon alignment telemetry could not initialize: {}", error.what());
        }
    }

    void shutdown()
    {
        ownerThread.store(0, std::memory_order_release);
        if (session) {
            session->log->info("VWA end overruns={}", session->pool->overrun_counter());
            session->log->flush();
            session.reset();
        }
    }

    void capture(Phase phase, std::uint64_t schedulerSequence)
    {
        if (!session || session->failed) {
            return;
        }
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled) {
            session->sampling = false;
            return;
        }
        auto* equipped = f4vr::getEquippedWeaponItem();
        const std::uint32_t formId = equipped ? equipped->item.object->formID : 0;
        if (phase == Phase::BeforeFrik) {
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
        const auto traversal = visitScene(static_cast<RE::NiAVObject*>(root), [&](RE::NiAVObject* current) {
            if (selected(current)) {
                if (emitted == 40) {
                    return false;
                }
                node(phaseLabel, "scene", current);
                ++emitted;
                const auto name = nodeName(current);
                if (name == "Weapon") sceneMask |= 1;
                if (name == "TGunReceiver" || name == "PipeRifleReceiver" || name == "RevolverReceiver") sceneMask |= 2;
                if (name == "ProjectileNode") sceneMask |= 4;
                if (name == "RArm_Hand") sceneMask |= 8;
                if (name == "LArm_Hand") sceneMask |= 16;
            }
            return true;
        });
        session->log->info("VWA phase-end seq={} phase={} root={:X} visited={} emitted={} truncated={} sceneMask={:X} weaponSceneComplete={}",
            schedulerSequence, phaseLabel, reinterpret_cast<std::uintptr_t>(root), traversal.visited, emitted,
            traversal.truncated, sceneMask, !traversal.truncated && (sceneMask & 7) == 7);
        if ((sceneMask & 7) != 7 || traversal.truncated) {
            session->log->warn("VWA incomplete-scene seq={} phase={} sceneMask={:X}; missing weapon/receiver/muzzle or traversal bound reached, do not infer transform ownership from this phase",
                schedulerSequence, phaseLabel, sceneMask);
        }
        if (phase == Phase::AfterRock) {
            handPresentation(phaseLabel);
            session->sampling = false;
            session->log->flush();
        }
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
        const RE::NiTransform& handInWeapon, const RE::NiTransform& trackedHand, const RE::NiTransform& solvedWeapon,
        const RE::NiTransform& solvedHand)
    {
        if (!sampling() || formId != session->formId) {
            return;
        }
        session->log->info("VWA authored-solve seq={} form={:08X} capture={} source={}",
            session->sequence, formId, captureSequence, source);
        transform("authored-solve", "hand-in-weapon", handInWeapon);
        transform("authored-solve", "tracked-hand-world", trackedHand);
        transform("authored-solve", "proposed-weapon-world", solvedWeapon);
        transform("authored-solve", "proposed-hand-world", solvedHand);
    }

    std::uint64_t beginNativeArm(ArmCaller caller, std::uintptr_t returnAddress,
        RE::NiNode** weapon, RE::NiNode** offset) noexcept
    {
        if (!diagnosticAllowed()) return 0;
        try {
            const auto index = static_cast<std::size_t>(caller);
            if (index >= session->armGates.size() || !admit(session->armGates[index], equippedForm())) return 0;
            const auto event = ++session->event;
            HMODULE module = nullptr;
            std::array<char, MAX_PATH> path{};
            if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                    reinterpret_cast<LPCSTR>(returnAddress), &module)) {
                GetModuleFileNameA(module, path.data(), static_cast<DWORD>(path.size()));
                path.back() = '\0';
            }
            session->log->info("VWA arm-caller event={} return={:X} module='{}' rva={:X}",
                event, returnAddress, path.data(), module ? returnAddress - reinterpret_cast<std::uintptr_t>(module) : 0);
            if (guardedArmScene(event, caller, true, weapon, offset)) return event;
        } catch (...) {}
        disableAfterFailure();
        return 0;
    }

    void endNativeArm(std::uint64_t event, ArmCaller caller, RE::NiNode** weapon, RE::NiNode** offset) noexcept
    {
        if (!event || !diagnosticAllowed()) return;
        try {
            if (guardedArmScene(event, caller, false, weapon, offset)) return;
        } catch (...) {}
        disableAfterFailure();
    }

    bool wantsAnimationSample(std::uint32_t formId) noexcept
    {
        return diagnosticAllowed() && admit(session->animationGate, formId);
    }

    void recordAnimationSample(std::uint32_t formId, std::uint64_t variant,
        std::uint64_t instance, std::uint64_t graphProfile, std::string_view clip,
        int animationType, std::uint32_t blendHint, std::span<const AnimationBone> bones) noexcept
    {
        if (!diagnosticAllowed() || !targeted(formId)) return;
        try {
            const auto event = ++session->event;
            session->log->info("VWA animation event={} form={:08X} variant={:016X} instance={:016X} graph={:016X} clip='{}' type={} blendHint={} sampleTime=0",
                event, formId, variant, instance, graphProfile, clip.substr(0, 260), animationType, blendHint);
            for (const auto& bone : bones.first((std::min)(bones.size(), std::size_t{ 12 }))) {
                session->log->info("VWA animation-bone event={} name='{}' bone={} parent={} track={} sampledValid={} referenceValid={} fromReference={}",
                    event, bone.name ? bone.name : "missing", bone.bone, bone.parent, bone.track,
                    bone.sampledValid, bone.referenceValid, bone.fromReference);
                if (bone.sampledValid) transform("animation-sample", "sampled-local", bone.sampled, event);
                if (bone.referenceValid) transform("animation-sample", "reference-local", bone.reference, event);
            }
        } catch (...) { disableAfterFailure(); }
    }
}
