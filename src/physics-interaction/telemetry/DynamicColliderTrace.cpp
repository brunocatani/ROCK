#include "physics-interaction/telemetry/DynamicColliderTrace.h"
#include "physics-interaction/telemetry/HeldRenderTrace.h"
#include "physics-interaction/grab/GrabMotorTelemetry.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/ProcessLists.h"
#include "RE/Havok/hknpBody.h"

#include "RockConfig.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "rock_support/Logger.h"
#include "rock_support/ResourceUtils.h"

#include <atomic>
#include <array>
#include <cmath>
#include <cstring>
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
            std::uint64_t nextNpcSampleMilliseconds = 0; // Game thread only.
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
            next->log->info("MOTOR_LOAD start version=2 layoutVerified={} interval=0.1-simulation-seconds axes=angular012,linear012 utilization=net-impulse/(limit*solver-dt) nearLimit=0.95 recoveryState=native-not-impulse observational=true",
                next->motorLayoutVerified);
            next->weaponLog->info("MOTOR_LOAD start version=2 layoutVerified={} interval=0.1-simulation-seconds axes=angular012,linear012 utilization=net-impulse/(limit*solver-dt) nearLimit=0.95 recoveryState=native-not-impulse observational=true",
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

    void captureNpcCollisionState(const PhysicsFrameContext& frame) noexcept
    {
        if (!enabled() || !g_rockConfig.npcDynamicCollisions ||
            !frame.hknpWorld || !frame.hasHmdFrame || frame.menuBlocked) return;
        const auto now = GetTickCount64();
        if (now < session->nextNpcSampleMilliseconds) return;
        session->nextNpcSampleMilliseconds = now + 500;

        auto* processes = RE::ProcessLists::GetSingleton();
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!processes || !player) return;
        RE::NiPointer<RE::Actor> nearest;
        float distanceSquared = 200.0f * 200.0f;
        const auto actorCount = processes->highActorHandles.size();
        const auto actorLimit = (std::min)(std::size_t(actorCount), std::size_t{128});
        for (decltype(processes->highActorHandles.size()) i = 0; i < actorLimit; ++i) {
            auto actor = processes->highActorHandles[i].get();
            if (!actor || actor.get() == player || actor->IsDeleted() || actor->IsDisabled() || !actor->Get3D()) continue;
            const auto position = actor->GetPosition();
            const auto delta = position - frame.hmdPositionWorld;
            const float distance = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            if (std::isfinite(distance) && distance < distanceSquared) {
                distanceSquared = distance;
                nearest = actor;
            }
        }
        if (!nearest) {
            writeWeapon("NPC_COLLISION_SCAN nearby=none actorCount={} actorScanTruncated={}", actorCount, actorCount > actorLimit);
            return;
        }

        struct BodySample
        {
            std::uint32_t id = 0, filter = 0, flags = 0, motion = 0;
            std::int32_t nativeSlot6C = -1;
            bool readable = false, poseBound = false, ownerMatches = false, shapePresent = false;
            bool rightPair = false, leftPair = false, weaponPair = false;
            RE::NiPoint3 position{};
        };
        struct Census
        {
            RE::hknpWorld* world = nullptr;
            std::array<BodySample, 64> bodies{};
            std::size_t count = 0;
            bool poseBound = false, truncated = false;

            static bool visit(std::uint32_t bodyId, void* context)
            {
                auto& self = *static_cast<Census*>(context);
                for (std::size_t i = 0; i < self.count; ++i) {
                    if (self.bodies[i].id == bodyId) return true;
                }
                if (self.count == self.bodies.size()) { self.truncated = true; return false; }
                auto& result = self.bodies[self.count++];
                result.id = bodyId;
                result.poseBound = self.poseBound;
                const auto identity = havok_runtime::snapshotBodyIdentity(self.world, RE::hknpBodyId{bodyId});
                std::array<std::byte, sizeof(RE::hknpBody)> body{};
                if (!identity.valid || !native_memory::guardedCopyFromMemory(identity.body, body.data(), body.size())) return true;
                const auto read = [&]<class T>(std::size_t offset, T& value) {
                    std::memcpy(&value, body.data() + offset, sizeof(value));
                };
                std::uint32_t liveId = 0;
                read(offsetof(RE::hknpBody, bodyId), liveId);
                if (liveId != bodyId) return true;
                result.readable = true;
                read(offsetof(RE::hknpBody, collisionFilterInfo), result.filter);
                read(offsetof(RE::hknpBody, flags), result.flags);
                read(offsetof(RE::hknpBody, motionIndex), result.motion);
                std::uintptr_t shape = 0;
                read(offsetof(RE::hknpBody, shape), shape);
                result.shapePresent = shape != 0;
                // Raw diagnostic value only: 153AF6C and 153C601/153C6B6 test
                // body+6C against -1. Do not infer registration from its name
                // in CommonLib; retain it alongside flags and scan status.
                read(offsetof(RE::hknpBody, deactivationIslandId), result.nativeSlot6C);
                std::array<float, 4> translation{};
                read(offsetof(RE::hknpBody, translation), translation);
                const auto scale = physics_scale::havokToGame();
                result.position = {translation[0] * scale, translation[1] * scale, translation[2] * scale};
                RE::hknpWorld* ownerWorld = nullptr;
                RE::hknpBodyId ownerBody{};
                result.ownerMatches = identity.collisionObject &&
                    havok_runtime::tryResolveCollisionObjectBody(identity.collisionObject, ownerWorld, ownerBody) &&
                    ownerWorld == self.world && ownerBody.value == bodyId;
                const auto* matrix = havok_runtime::getCollisionFilterMatrix(self.world);
                const auto layer = result.filter & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                using namespace collision_layer_policy;
                result.rightPair = matrix && layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY, layer, true);
                result.leftPair = matrix && layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, layer, true);
                result.weaponPair = matrix && layerPairSymmetricMatches(matrix, ROCK_LAYER_DYNAMIC_WEAPON_PROXY, layer, true);
                return true;
            }
        } census{ .world = frame.hknpWorld };
        havok_runtime::PhysicsSystemBodyScanResult poseScan{};
        std::size_t visitedNodes = 0, collisionObjects = 0, failedObjects = 0;
        std::uint32_t sceneStatusMask = 0, sceneInvalidIds = 0;
        bool posePresent = false, nodeTruncated = false;
        {
            // Existing world read discipline; no NPC writes, activation, or
            // retained engine pointers. Logs are published after unlocking.
            havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
            auto* process = nearest->currentProcess;
            auto* middleHigh = process ? process->middleHigh : nullptr;
            auto* pose = middleHigh ? middleHigh->poseBound.get() : nullptr;
            posePresent = pose != nullptr;
            if (pose) {
                census.poseBound = true;
                poseScan = havok_runtime::forEachPhysicsSystemBodyIdDetailed(pose, frame.hknpWorld, 64, Census::visit, &census);
                census.truncated |= poseScan.bodyCount > 64;
                census.poseBound = false;
            }
            std::array<RE::NiAVObject*, 512> nodes{};
            std::size_t queued = 0;
            if (auto* root = nearest->Get3D()) nodes[queued++] = root;
            while (visitedNodes < queued && collisionObjects < 64) {
                auto* node = nodes[visitedNodes++];
                if (auto* collision = node->collisionObject.get()) {
                    ++collisionObjects;
                    const auto scan = havok_runtime::forEachPhysicsSystemBodyIdDetailed(collision, frame.hknpWorld, 64, Census::visit, &census);
                    if (!scan.enumerated()) ++failedObjects;
                    sceneStatusMask |= 1u << static_cast<unsigned>(scan.status);
                    sceneInvalidIds += scan.skippedInvalidBodies;
                    census.truncated |= scan.bodyCount > 64;
                }
                if (auto* branch = node->IsNode()) {
                    const auto& children = branch->GetRuntimeData().children;
                    const auto childLimit = (std::min)(std::size_t(children.size()), nodes.size());
                    nodeTruncated |= children.size() > childLimit;
                    for (decltype(children.size()) i = 0; i < childLimit; ++i) {
                        if (auto* child = children[i].get()) {
                            if (queued == nodes.size()) { nodeTruncated = true; break; }
                            nodes[queued++] = child;
                        }
                    }
                }
            }
            nodeTruncated |= visitedNodes < queued;
        }
        writeWeapon("NPC_COLLISION_SCAN actor={:08X} dead={} distance={:.1f} posePresent={} poseStatus={} poseBodies={} poseInvalidIds={} sceneNodes={} sceneObjects={} sceneFailures={} sceneStatusMask={:X} sceneInvalidIds={} bodies={} truncated={} actorScanTruncated={}",
            nearest->GetFormID(), nearest->IsDead(false), std::sqrt(distanceSquared), posePresent,
            havok_runtime::physicsSystemBodyScanStatusName(poseScan.status), poseScan.bodyCount, poseScan.skippedInvalidBodies,
            visitedNodes, collisionObjects, failedObjects, sceneStatusMask, sceneInvalidIds,
            census.count, census.truncated || nodeTruncated, actorCount > actorLimit);
        for (std::size_t i = 0; i < census.count; ++i) {
            const auto& body = census.bodies[i];
            writeWeapon("NPC_COLLISION_BODY actor={:08X} source={} body={} readable={} ownerMatches={} layer={} filter={:08X} flags={:08X} motion={} nativeSlot6C={} shape={} pairs(R,L,W)=({},{},{}) position=({:.2f},{:.2f},{:.2f})",
                nearest->GetFormID(), body.poseBound ? "pose" : "scene", body.id, body.readable, body.ownerMatches,
                body.filter & collision_layer_policy::FO4_LAYER_FILTER_MASK, body.filter, body.flags, body.motion,
                body.nativeSlot6C, body.shapePresent, body.rightPair, body.leftPair, body.weaponPair,
                body.position.x, body.position.y, body.position.z);
        }
    }

    void suppressAfterError() noexcept
    {
        writerFailed.store(true, std::memory_order_relaxed);
        recording.store(false, std::memory_order_release);
        presentationRecording.store(false, std::memory_order_release);
    }
}
