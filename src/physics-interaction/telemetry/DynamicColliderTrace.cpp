#include "physics-interaction/telemetry/DynamicColliderTrace.h"
#include "physics-interaction/telemetry/HeldRenderTrace.h"
#include "physics-interaction/grab/GrabMotorTelemetry.h"

#include "RockConfig.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "rock_support/Logger.h"
#include "rock_support/ResourceUtils.h"

#include <atomic>
#include <memory>
#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <Windows.h>

namespace rock::dynamic_collider_trace
{
    namespace
    {
        struct Session
        {
            // The logger dies before the pool drains and joins its sole worker.
            // The worker receives formatted messages, never engine pointers.
            std::shared_ptr<spdlog::details::thread_pool> pool;
            std::shared_ptr<spdlog::async_logger> log;
            std::shared_ptr<spdlog::async_logger> weaponLog;
            bool motorLayoutVerified = false;
        };
        // Lifecycle changes are game-thread-only, outside active physics callbacks.
        std::unique_ptr<Session> session;
        std::atomic<bool> recording{ false };
        std::atomic<bool> presentationRecording{ false };
        std::atomic<bool> writerFailed{ false };
    }

    void initialize() noexcept
    {
        if (session || (!g_rockConfig.rockDebugGrabFrameLogging && !g_rockConfig.rockDebugShowSkeletonBoneVisualizer &&
                !g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers)) return;
        try {
            auto next = std::make_unique<Session>();
            const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_ColliderTrace.log");
            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 10 * 1024 * 1024, 5, true);
            next->pool = std::make_shared<spdlog::details::thread_pool>(2048, 1);
            next->log = std::make_shared<spdlog::async_logger>("ROCK_ColliderTrace", sink, next->pool,
                spdlog::async_overflow_policy::overrun_oldest);
            next->log->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->log->set_error_handler([](const std::string&) { suppressAfterError(); });
            // Dense hand traffic must not evict an earlier weapon-contact
            // reproduction from the same session. Reuse the existing worker.
            const auto weaponPath = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_WeaponContact.log");
            auto weaponSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(weaponPath, 10 * 1024 * 1024, 5, true);
            next->weaponLog = std::make_shared<spdlog::async_logger>("ROCK_WeaponContact", weaponSink, next->pool,
                spdlog::async_overflow_policy::overrun_oldest);
            next->weaponLog->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->weaponLog->set_error_handler([](const std::string&) { suppressAfterError(); });
            writerFailed.store(false, std::memory_order_relaxed);
            next->motorLayoutVerified = grab_motor_telemetry::verifyLayout();
            next->log->info("MOTOR_LOAD start version=1 layoutVerified={} interval=0.1-simulation-seconds axes=angular012,linear012 utilization=net-impulse/(limit*solver-dt) nearLimit=0.95 recoveryState=native-not-impulse observational=true",
                next->motorLayoutVerified);
            next->weaponLog->info("MOTOR_LOAD start version=1 layoutVerified={} interval=0.1-simulation-seconds axes=angular012,linear012 utilization=net-impulse/(limit*solver-dt) nearLimit=0.95 recoveryState=native-not-impulse observational=true",
                next->motorLayoutVerified);
            next->log->info("COLLIDER_TRACE start version=5 pid={} build={} {} sourceStride=4 weaponBurst=12/120 heldPhaseBurst=12/120 observational=true positions=game-units rotations=quaternion-xyzw velocities=havok-units-per-second peerKind=1:hand,2:weapon,3:world",
                GetCurrentProcessId(), __DATE__, __TIME__);
            next->log->flush();
            next->weaponLog->info("WEAPON_CONTACT_TRACE start version=1 pid={} build={} {} sourceStride=4 weaponBurst=12/120 leverBaselineStride=120 observational=true positions=game-units rotations=quaternion-xyzw",
                GetCurrentProcessId(), __DATE__, __TIME__);
            next->weaponLog->flush();
            session = std::move(next);
            recording.store(g_rockConfig.rockDebugGrabFrameLogging, std::memory_order_release);
            presentationRecording.store(true, std::memory_order_release);
            held_render_trace::initialize();
        } catch (...) {
            suppressAfterError();
            try { logger::error("ROCK: Collider trace initialization failed."); } catch (...) {}
        }
    }

    void shutdown() noexcept
    {
        held_render_trace::shutdown();
        recording.store(false, std::memory_order_release);
        presentationRecording.store(false, std::memory_order_release);
        if (!session) return;
        try {
            session->log->info("COLLIDER_TRACE end overruns={} writerFailed={}",
                session->pool->overrun_counter(), writerFailed.load(std::memory_order_relaxed));
            session->log->flush();
            session->weaponLog->info("WEAPON_CONTACT_TRACE end overruns={} writerFailed={}",
                session->pool->overrun_counter(), writerFailed.load(std::memory_order_relaxed));
            session->weaponLog->flush();
        } catch (...) {
            try { logger::error("ROCK: Collider trace final status failed."); } catch (...) {}
        }
        session.reset();
    }

    void beginFrame(bool requested, std::uint64_t frame) noexcept
    {
        recording.store(requested && session && !writerFailed.load(std::memory_order_relaxed), std::memory_order_release);
        presentationRecording.store((requested || g_rockConfig.rockDebugShowSkeletonBoneVisualizer ||
            g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers) && session && !writerFailed.load(std::memory_order_relaxed), std::memory_order_release);
        if (!presentationEnabled() || frame % 300 != 0) return;
        write("COLLIDER_TRACE heartbeat frame={} overruns={}", frame, session->pool->overrun_counter());
        try {
            session->log->flush();
            session->weaponLog->flush();
        } catch (...) { suppressAfterError(); }
    }

    bool enabled() noexcept { return recording.load(std::memory_order_acquire); }
    bool motorOutputEnabled() noexcept { return enabled() && session->motorLayoutVerified; }
    bool presentationEnabled() noexcept { return presentationRecording.load(std::memory_order_acquire); }
    bool sample(std::uint64_t sequence) noexcept { return enabled() && sequence != 0 && sequence % 4 == 0; }
    spdlog::logger* activeLogger() noexcept { return presentationEnabled() ? session->log.get() : nullptr; }
    spdlog::logger* activeWeaponLogger() noexcept { return enabled() ? session->weaponLog.get() : nullptr; }
    void capturePresentedHands(std::uint64_t frame) noexcept
    {
        if (!sample(frame)) return;
        for (const bool left : { false, true }) {
            RE::NiTransform driver{}, raw{}, presented{};
            const bool driverValid = frik_hand_world_authority::tryGetInputDriverWorld(left, driver);
            const bool rawValid = frik_hand_world_authority::tryGetRawHandWorld(left, raw);
            const bool presentedValid = frik_hand_world_authority::tryGetPresentedHandWorld(left, presented);
            write("DHC_CLOCK frame-end: frame={} hand={} driverValid={} rawValid={} presentedValid={} driver=({:.4f},{:.4f},{:.4f}) raw=({:.4f},{:.4f},{:.4f}) presented=({:.4f},{:.4f},{:.4f})",
                frame, left ? "L" : "R", driverValid, rawValid, presentedValid,
                driver.translate.x, driver.translate.y, driver.translate.z,
                raw.translate.x, raw.translate.y, raw.translate.z,
                presented.translate.x, presented.translate.y, presented.translate.z);
        }
    }
    void suppressAfterError() noexcept
    {
        writerFailed.store(true, std::memory_order_relaxed);
        recording.store(false, std::memory_order_release);
        presentationRecording.store(false, std::memory_order_release);
    }
}
