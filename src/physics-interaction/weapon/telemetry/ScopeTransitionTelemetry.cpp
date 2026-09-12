#include "physics-interaction/weapon/telemetry/ScopeTransitionTelemetry.h"
#include "physics-interaction/weapon/telemetry/ScopeTransitionTracePolicy.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"

#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <atomic>
#include <memory>
#include <string_view>
#include <Windows.h>

namespace rock::scope_transition_telemetry
{
    namespace
    {
        namespace policy = scope_transition_trace_policy;
        // The existing UI sink only publishes a coherent event value. All
        // session, graph and prediction access stays on the game thread.
        std::atomic<std::uint64_t> menuRevision{ 0 }, menuEvent{ 0 };
        std::atomic<bool> writerFailed{ false };
        struct Session
        {
            // Destruction drains the writer before releasing its pool. The
            // worker owns formatted messages only and cannot read the game.
            std::shared_ptr<spdlog::details::thread_pool> pool;
            std::shared_ptr<spdlog::async_logger> log;
            policy::Window window{};
            bool failed = false;
        };
        std::unique_ptr<Session> session;

        const char* phaseName(Phase phase)
        {
            switch (phase) {
            case Phase::BeforeFrik: return "before-frik";
            case Phase::AfterFrik: return "after-frik";
            case Phase::AfterRock: return "after-rock";
            }
            return "unknown";
        }

        void pose(const char* phase, const char* label, const RE::NiTransform& value, bool valid)
        {
            const auto& r = value.rotate.entry;
            const bool finite = hand_world_claim_registry_policy::isFiniteTransform(value);
            write("SCT pose frame={} phase={} label={} valid={} finite={} T=({:.4f},{:.4f},{:.4f}) S={:.6f} R=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f})",
                sequence(), phase, label, valid, finite, value.translate.x, value.translate.y, value.translate.z, value.scale,
                r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]);
        }

        void node(const char* phase, const char* label, const RE::NiAVObject* value)
        {
            write("SCT node frame={} phase={} label={} ptr={:X} parent={:X} localT=({:.4f},{:.4f},{:.4f}) localScale={:.6f}",
                sequence(), phase, label, reinterpret_cast<std::uintptr_t>(value),
                reinterpret_cast<std::uintptr_t>(value ? value->parent : nullptr),
                value ? value->local.translate.x : 0.0f, value ? value->local.translate.y : 0.0f,
                value ? value->local.translate.z : 0.0f, value ? value->local.scale : 0.0f);
            pose(phase, label, value ? value->world : RE::NiTransform{}, value != nullptr);
        }

        void scene(const char* phase, RE::NiNode* root, bool firstPerson)
        {
            // No cross-frame node cache. Limit both sparse child traversal
            // and matching-node output, including duplicate bone names.
            unsigned mask = 0, emitted = 0;
            const auto traversal = weapon_scene::visitScene(
                static_cast<RE::NiAVObject*>(root), [&](RE::NiAVObject* current) {
                    const char* rawName = current->name.c_str();
                    const std::string_view name = rawName ? std::string_view(rawName) : std::string_view{};
                    unsigned bit = 0;
                    const char* label = nullptr;
                    if (name == "RArm_Hand") { bit = 1; label = firstPerson ? "fp-right-hand" : "body-right-hand"; }
                    if (name == "LArm_Hand") { bit = 2; label = firstPerson ? "fp-left-hand" : "body-left-hand"; }
                    if (firstPerson && name == "Weapon") { bit = 4; label = "weapon"; }
                    if (label) {
                        if (emitted == 6) return false;
                        node(phase, label, current);
                        mask |= bit;
                        ++emitted;
                    }
                    return true;
                });
            write("SCT scene frame={} phase={} firstPerson={} root={:X} visited={} emitted={} mask={} truncated={}",
                sequence(), phase, firstPerson, reinterpret_cast<std::uintptr_t>(root), traversal.visited, emitted, mask, traversal.truncated);
        }

        void dampening(const char* phase, const frik_hand_world_authority::ScopeDampenTrace& d)
        {
            write("SCT dampen frame={} phase={} driverSequence={} observedSequence={} runtimeFrameObserved={} runtimeMenuSnapshot={} menuUsed={} enabled={} factors=({:.3f},{:.3f}) cameraValid=({},{}) cameraNow=({:.4f},{:.4f},{:.4f}) cameraPrevious=({:.4f},{:.4f},{:.4f})",
                sequence(), phase, d.driverSequence, d.observedSequence, d.runtimeFrameObserved, d.runtimeMenuSnapshot, d.menuUsed, d.enabled,
                d.translationFactor, d.rotationFactor, d.cameraNowValid, d.cameraPreviousValid,
                d.cameraNow.x, d.cameraNow.y, d.cameraNow.z, d.cameraPrevious.x, d.cameraPrevious.y, d.cameraPrevious.z);
            for (std::size_t i = 0; i < 2; ++i) {
                const bool left = i == hand_world_claim_registry_policy::handIndex(true);
                const auto& camera = d.historyCamera[i];
                const auto movement = d.cameraNow - camera;
                const bool historyKnown = d.historySequence[i] != 0 && d.historySequence[i] <= sequence();
                write("SCT prediction frame={} phase={} hand={} mode={} errorValid={} errorGameUnits={:.4f} errorDegrees={:.4f}",
                    sequence(), phase, left ? "left" : "right", dampened_driver_prediction_policy::predictionModeName(d.predictionMode[i]),
                    d.predictionErrorValid[i], d.predictionTranslationError[i], d.predictionRotationError[i]);
                write("SCT input frame={} phase={} hand={} isolated={} recoveryMask={} relationInputValid={}",
                    sequence(), phase, left ? "left" : "right", d.inputIsolated[i], d.recoveryMask, d.firstPersonInput[i].valid);
                write("SCT history frame={} phase={} hand={} historySequence={} ageKnown={} ageFrames={} cameraValid={} origin=({:.4f},{:.4f},{:.4f}) accumulatedCamera=({:.4f},{:.4f},{:.4f})",
                    sequence(), phase, left ? "left" : "right", d.historySequence[i], historyKnown,
                    historyKnown ? sequence() - d.historySequence[i] : 0, d.historyCameraValid[i] && d.cameraNowValid,
                    camera.x, camera.y, camera.z, movement.x, movement.y, movement.z);
                pose(phase, left ? "raw-left" : "raw-right", d.raw[i].world, d.raw[i].valid);
                pose(phase, left ? "effective-driver-left" : "effective-driver-right", d.driver[i].world, d.driver[i].valid);
                pose(phase, left ? "native-driver-left" : "native-driver-right", d.nativeDriver[i].world, d.nativeDriver[i].valid);
                pose(phase, left ? "input-fp-left" : "input-fp-right", d.firstPersonInput[i].world, d.firstPersonInput[i].valid);
                pose(phase, left ? "history-left" : "history-right", d.history[i].world, d.history[i].valid);
                pose(phase, left ? "cached-presented-left" : "cached-presented-right", d.presented[i].world, d.presented[i].valid);
                pose(phase, left ? "consumed-left" : "consumed-right", d.consumed[i].target, d.consumed[i].valid);
                pose(phase, left ? "claimed-left" : "claimed-right", d.claimed[i].target, d.claimed[i].valid);
            }
        }
    }

    void onMenuEvent(bool open) noexcept
    {
        const auto revision = menuRevision.fetch_add(1, std::memory_order_relaxed) + 1;
        menuEvent.store((revision << 1) | static_cast<std::uint64_t>(open), std::memory_order_release);
    }

    void initialize() noexcept
    {
        if (session || !g_rockConfig.rockDebugHandWorldAuthority) return;
        try {
            auto next = std::make_unique<Session>();
            const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_ScopeTransitions.log");
            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 8 * 1024 * 1024, 3, true);
            next->pool = std::make_shared<spdlog::details::thread_pool>(2048, 1);
            next->log = std::make_shared<spdlog::async_logger>("ROCK_ScopeTransitions", sink, next->pool,
                spdlog::async_overflow_policy::overrun_oldest);
            next->log->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->log->set_error_handler([](const std::string&) { writerFailed.store(true, std::memory_order_relaxed); });
            writerFailed.store(false, std::memory_order_relaxed);
            next->log->info("SCT start version=3 pid={} build={} {} tailFrames={} maximumBurstFrames={} matrices=Ni-stored-rows scopeTraceObservational=true",
                GetCurrentProcessId(), __DATE__, __TIME__, policy::kTailFrames, policy::kMaximumBurstFrames);
            next->log->flush();
            session = std::move(next);
            logger::info("ROCK: Scope transition telemetry enabled at '{}'.", path);
        } catch (...) {
            try { logger::error("ROCK: Scope transition telemetry initialization failed."); } catch (...) {}
        }
    }

    void shutdown() noexcept
    {
        if (!session) return;
        try {
            session->log->info("SCT end overruns={} writerFailed={}", session->pool->overrun_counter(), writerFailed.load());
            session->log->flush();
        } catch (...) {
            try { logger::error("ROCK: Scope transition trace could not write its final status."); } catch (...) {}
        }
        session.reset(); // Skeleton lifetime boundary, never a frame callback.
    }

    spdlog::logger* activeLogger() noexcept
    {
        return session && !session->failed && session->window.active && g_rockConfig.rockDebugHandWorldAuthority ? session->log.get() : nullptr;
    }

    std::uint64_t sequence() noexcept { return session ? session->window.frame : 0; }

    void suppressAfterError() noexcept
    {
        if (!session || session->failed) return;
        session->failed = true;
        try { logger::error("ROCK: Scope transition capture disabled after a logging failure; diagnostic data is incomplete."); } catch (...) {}
    }

    void capture(Phase phase, std::uint64_t schedulerSequence) noexcept
    {
        if (!session || session->failed) return;
        if (!g_rockConfig.rockDebugHandWorldAuthority) { session->window = {}; return; }
        if (writerFailed.load(std::memory_order_relaxed)) { suppressAfterError(); return; }
        try {
            std::uint8_t renderer = 0;
            const bool rendererValid = native_memory::tryReadField(
                reinterpret_cast<const void*>(REL::Offset(offsets::kData_NativeScopeRendererState).address()), 3, renderer);
            const policy::Signals signals{ menuEvent.load(std::memory_order_acquire),
                input_remap_runtime::isManualScopeActivationRequested(), rendererValid, renderer != 0,
                frik_hand_world_authority::scopeInputRecoveryMask() };
            const bool wasCapped = session->window.capped;
            if (!session->window.observe(schedulerSequence, static_cast<unsigned>(phase), signals)) {
                if (!wasCapped && session->window.capped) session->log->warn("SCT burst capped frame={} edge={}; waiting for quiet gap", schedulerSequence, session->window.edge);
                return;
            }
            const auto* label = phaseName(phase);
            const auto& runtime = runtime_state::currentFrame();
            const auto* equipped = f4vr::getEquippedWeaponItem();
            const auto* player = f4vr::getPlayer();
            write("SCT phase frame={} phase={} edge={} phaseMask={} menuEvent={} menuKnown={} menuOpen={} button={} rendererValid={} renderer={} runtimeFrame={} runtimeScope={} dt={:.6f} form={:08X} gunState={} overruns={}",
                schedulerSequence, label, session->window.edge, session->window.phaseMask, signals.menuEvent >> 1,
                signals.menuEvent != 0, (signals.menuEvent & 1) != 0, signals.button, signals.rendererValid, signals.renderer,
                runtime.frameIndex, runtime.localScopeMenuOpen, runtime.deltaSeconds,
                equipped && equipped->item.object ? equipped->item.object->formID : 0, f4vr::getNativeGunState(player), session->pool->overrun_counter());
            pose(label, "runtime-player-space", runtime.playerSpace.world, runtime.playerSpace.valid);
            auto* nodes = f4vr::getPlayerNodes();
            auto* camera = f4vr::getPlayerCamera();
            node(label, "hmd", nodes ? nodes->HmdNode : nullptr);
            node(label, "view-camera-root", camera ? camera->cameraRoot.get() : nullptr);
            node(label, "room", nodes ? nodes->roomnode : nullptr);
            node(label, "player-world", nodes ? nodes->playerworldnode : nullptr);
            node(label, "scope-camera", nodes ? nodes->primaryWeaponScopeCamera : nullptr);
            node(label, "scope-parent", nodes ? nodes->ScopeParentNode : nullptr);
            node(label, "body-root", f4vr::getRootNode());
            node(label, "right-wand", nodes ? nodes->primaryWandNode : nullptr);
            node(label, "left-wand", nodes ? nodes->SecondaryWandNode : nullptr);
            node(label, "right-driver", nodes ? nodes->primaryWeaponOffsetNOde : nullptr);
            node(label, "left-driver", nodes ? nodes->SecondaryMeleeWeaponOffsetNode2 : nullptr);
            scene(label, f4vr::getFirstPersonSkeleton(), true);
            scene(label, f4vr::getRootNode(), false);
            if (phase == Phase::AfterFrik && (session->window.phaseMask & 1) == 0) {
                // A UI event can arrive inside FRIK's pass, after the pre
                // scene opportunity. Retain the actual predictor inputs even
                // on that edge; never invent a pre-event scene transform.
                const auto input = frik_hand_world_authority::scopeDampenTraceBeforeFrik();
                const bool inputCurrent = input.driverSequence == sequence();
                write("SCT late-edge frame={} beforeSceneCaptured=false predictorInputRetained={}", sequence(), inputCurrent);
                if (inputCurrent) dampening("prediction-input-before-rebase", input);
            }
            dampening(label, frik_hand_world_authority::scopeDampenTrace());
            if (phase == Phase::AfterRock) session->log->flush(); // Queued, no disk wait on the game thread.
        } catch (...) { suppressAfterError(); }
    }
}
