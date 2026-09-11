#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/ResourceUtils.h"
#include "RockConfig.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <Windows.h>

namespace rock::native_scope_shot_diagnostics
{
    namespace
    {
        namespace policy = native_scope_shot_policy;
        using policy::Point;
        using policy::Ray;
        enum class ReadStage : std::uint8_t { None, Owner, Node, WorldBytes, FiniteWorld };

        // All callbacks make one nonblocking attempt. Every payload access
        // owns the flag, including teardown. No seqlock over non-atomic data.
        template<class T> struct Channel
        {
            std::atomic_flag busy = ATOMIC_FLAG_INIT;
            T value{};
            bool store(const T& input) noexcept
            {
                if (busy.test_and_set(std::memory_order_acquire)) return false;
                value = input;
                busy.clear(std::memory_order_release);
                return true;
            }
            bool read(T& output) noexcept
            {
                if (busy.test_and_set(std::memory_order_acquire)) return false;
                output = value;
                busy.clear(std::memory_order_release);
                return true;
            }
        };

        struct Frame
        {
            std::uint64_t index{}, milliseconds{}, generation{}, epoch{};
            std::uint32_t form{};
            ReadStage scopeReadStage{}, muzzleReadStage{};
            Ray scope{}, muzzle{};
            Point right{}, up{}, aimPoint{};
            bool valid{}, aimValid{}, scoped{}, contactKnown{}, contact{}, bipodMode{}, bipodLatched{};
        };

        // Raw launch prefix, not the propagated CommonLib class layout.
        // FO4VR 0x140333740 fills shooter/weapon/equip fields and passes this
        // same stack object to 0x14104F3A0 then 0x14104E860. Raw producer
        // 0x14104F3A0 and consumer 0x141057B00 agree on origin and angles.
        struct LaunchPrefix
        {
            Point origin{}, normal{};
            std::uintptr_t projectile{}, shooter{}, combat{}, weapon{}, instance{}, ammo{};
            std::uint32_t equip{};
            float yaw{}, pitch{}, roll{};
        };
        static_assert(offsetof(LaunchPrefix, shooter) == 0x20);
        static_assert(offsetof(LaunchPrefix, weapon) == 0x30);
        static_assert(offsetof(LaunchPrefix, yaw) == 0x4C);
        static_assert(offsetof(LaunchPrefix, pitch) == 0x50);
        static_assert(sizeof(LaunchPrefix) == 0x58);

        struct Shot
        {
            Frame presented{}, following{};
            Ray scopeAtSetup{}, muzzleAtSetup{}, nativeAim{}, initial{}, launch{};
            Point aimPoint{};
            std::uint64_t sequence{}, milliseconds{}, epoch{};
            std::uint32_t thread{}, form{}, equip{}, handle{}, stage{};
            ReadStage scopeReadStage{}, muzzleReadStage{};
            float yaw{}, pitch{};
            bool setupMatched{}, onGameThread{}, aimValid{}, launchReturned{};
        };
        struct Pending
        {
            Shot shot{};
            std::uintptr_t identity{}, shooter{}; // Comparison only, never retained for dereference.
            bool active{}, valid{};
        };
        thread_local Pending pending;
        Channel<Frame> presentedChannel;
        Channel<Shot> nativeChannel, completedChannel;
        std::atomic<bool> enabled{ false }, writerFailed{ false };
        std::atomic<std::uint32_t> gameThread{ 0 };
        std::atomic<std::uint64_t> epoch{ 1 }, nextSample{ 0 }, shotSequence{ 0 }, skipped{ 0 }, dropped{ 0 };
        std::atomic<std::uint64_t> playerSetups{ 0 }, muzzleReads{ 0 }, matchedLaunches{ 0 };
        Frame liveFrame{}; // Main thread only.
        Shot displayedShot{};
        std::uint64_t consumedSequence{};
        bool installed{}, writerErrorReported{};

        using SetOrigin = std::uint64_t (*)(void*);
        using FireNode = RE::NiAVObject* (*)(RE::Actor*, std::uint32_t);
        using Launch = std::uint32_t* (*)(std::uint32_t*, void*);
        SetOrigin originalSetOrigin{};
        FireNode originalFireNode{};
        Launch originalLaunch{};

        Point point(const RE::NiPoint3& p) noexcept { return { p.x, p.y, p.z }; }
        RE::NiPoint3 ni(Point p) noexcept { return { p.x, p.y, p.z }; }

        bool readWorld(const RE::NiAVObject* node, RE::NiTransform& world, ReadStage& stage) noexcept
        {
            const auto address = reinterpret_cast<std::uintptr_t>(node);
            if (!node || !native_memory::pointerLooksReadable(node) || (address & 7) != 0 || address > 0x00007FFFFFFFFFFF) return false;
            stage = ReadStage::Node;
            if (!native_memory::tryReadValue(&node->world, world)) return false;
            stage = ReadStage::WorldBytes;
            if (!policy::finite(point(world.translate)) || !std::isfinite(world.scale) || world.scale <= 0.0001f) return false;
            for (const auto& row : world.rotate.entry)
                for (int i = 0; i < 3; ++i) if (!std::isfinite(row[i])) return false;
            stage = ReadStage::FiniteWorld;
            return true;
        }
        Ray axisRay(const RE::NiTransform& world, unsigned axis) noexcept
        {
            const auto& r = world.rotate.entry[axis];
            return policy::ray(point(world.translate), { r[0], r[1], r[2] });
        }
        void readSight(Frame& frame) noexcept
        {
            auto* nodes = fo4vr::getPlayerNodes();
            RE::NiNode* camera{};
            RE::NiTransform world{};
            if (nodes) frame.scopeReadStage = ReadStage::Owner;
            if (nodes && native_memory::tryReadValue(&nodes->primaryWeaponScopeCamera, camera) && readWorld(camera, world, frame.scopeReadStage)) {
                // NiCamera +X is view forward; weapon ProjectileNode +Y is forward.
                frame.scope = axisRay(world, 0);
                frame.up = axisRay(world, 2).direction;
                const auto f = frame.scope.direction;
                const auto u = frame.up;
                frame.right = policy::ray({}, { f.y * u.z - f.z * u.y, f.z * u.x - f.x * u.z, f.x * u.y - f.y * u.x }).direction;
            }
            // Raw writer 0x140F7B6AA/B2/BA and reader 0x14104F7A6..D8
            // agree on this cached world-space native aiming point.
            frame.aimValid = native_memory::tryReadField(fo4vr::getPlayer(), 0xB90, frame.aimPoint) && policy::finite(frame.aimPoint);
        }

        std::uint64_t onSetOrigin(void* data)
        {
            if (!enabled.load(std::memory_order_acquire) || pending.active) return originalSetOrigin(data);
            pending = {};
            LaunchPrefix prefix{};
            const auto* player = fo4vr::getPlayer();
            if (!player || !native_memory::tryReadValue(static_cast<const LaunchPrefix*>(data), prefix) ||
                prefix.shooter != reinterpret_cast<std::uintptr_t>(player)) return originalSetOrigin(data);
            playerSetups.fetch_add(1, std::memory_order_relaxed);
            pending.identity = reinterpret_cast<std::uintptr_t>(data);
            pending.shooter = prefix.shooter;
            pending.shot.epoch = epoch.load(std::memory_order_acquire);
            pending.shot.thread = GetCurrentThreadId();
            pending.shot.onGameThread = pending.shot.thread == gameThread.load(std::memory_order_acquire);
            (void)presentedChannel.read(pending.shot.presented);
            pending.active = true;
            const auto result = originalSetOrigin(data);
            pending.active = false;
            if ((result & 0xFF) == 0 || !native_memory::tryReadValue(static_cast<const LaunchPrefix*>(data), prefix)) return result;
            pending.shot.initial = policy::launchRay(prefix.origin, prefix.yaw, prefix.pitch);
            pending.shot.equip = prefix.equip;
            if (prefix.weapon && native_memory::tryReadField(reinterpret_cast<const void*>(prefix.weapon), 0x14, pending.shot.form))
                pending.shot.stage |= 1;
            // The native hook may run on an animation worker. Only its own
            // arguments/returned fire node are sampled there; live camera and
            // player-cache reads belong to the established game thread.
            if (pending.shot.onGameThread) {
                Frame current{};
                readSight(current);
                pending.shot.scopeAtSetup = current.scope;
                pending.shot.scopeReadStage = current.scopeReadStage;
                pending.shot.aimPoint = current.aimPoint;
                pending.shot.aimValid = current.aimValid;
                if (current.scope.valid) pending.shot.stage |= 8;
                if (current.aimValid) pending.shot.stage |= 16;
                if (current.aimValid) pending.shot.nativeAim = policy::ray(prefix.origin, policy::subtract(current.aimPoint, prefix.origin));
            }
            pending.valid = pending.shot.initial.valid;
            return result;
        }

        RE::NiAVObject* onFireNode(RE::Actor* actor, std::uint32_t equip)
        {
            auto* node = originalFireNode(actor, equip);
            if (pending.active && reinterpret_cast<std::uintptr_t>(actor) == pending.shooter) {
                muzzleReads.fetch_add(1, std::memory_order_relaxed);
                RE::NiTransform world{};
                pending.shot.muzzleReadStage = ReadStage::Owner;
                if (readWorld(node, world, pending.shot.muzzleReadStage)) {
                    pending.shot.muzzleAtSetup = axisRay(world, 1);
                    pending.shot.stage |= 2;
                }
            }
            return node;
        }

        std::uint32_t* onLaunch(std::uint32_t* output, void* data)
        {
            Shot shot{};
            bool capture = enabled.load(std::memory_order_acquire) && pending.valid &&
                pending.identity == reinterpret_cast<std::uintptr_t>(data) &&
                pending.shot.epoch == epoch.load(std::memory_order_acquire);
            if (capture) {
                matchedLaunches.fetch_add(1, std::memory_order_relaxed);
                const auto now = GetTickCount64();
                auto deadline = nextSample.load(std::memory_order_relaxed);
                capture = now >= deadline && nextSample.compare_exchange_strong(deadline,
                    now + policy::kSampleMilliseconds, std::memory_order_relaxed);
                if (!capture) skipped.fetch_add(1, std::memory_order_relaxed);
                else {
                    LaunchPrefix prefix{};
                    shot = pending.shot;
                    capture = native_memory::tryReadValue(static_cast<const LaunchPrefix*>(data), prefix) && prefix.shooter == pending.shooter;
                    if (capture) {
                        shot.setupMatched = true;
                        shot.milliseconds = now;
                        shot.yaw = prefix.yaw;
                        shot.pitch = prefix.pitch;
                        shot.launch = policy::launchRay(prefix.origin, prefix.yaw, prefix.pitch);
                        shot.stage |= 4;
                    }
                }
            }
            // These are the final per-projectile angles, after the native
            // cone-of-fire loop, immediately before Launch consumes them.
            // Never alter the launch data, native result or firing behavior.
            auto* result = originalLaunch(output, data);
            if (capture) {
                shot.launchReturned = result && native_memory::tryReadValue(result, shot.handle);
                shot.sequence = shotSequence.fetch_add(1, std::memory_order_relaxed) + 1;
                if (!nativeChannel.store(shot)) dropped.fetch_add(1, std::memory_order_relaxed);
            }
            return result;
        }

        void logRay(spdlog::logger& log, std::uint64_t sequence, const char* phase, const char* name, const Ray& r)
        {
            log.info("SSA ray shot={} phase={} name={} valid={} origin=({:.6f},{:.6f},{:.6f}) direction=({:.9f},{:.9f},{:.9f})",
                sequence, phase, name, r.valid, r.origin.x, r.origin.y, r.origin.z, r.direction.x, r.direction.y, r.direction.z);
        }
        void logFrame(spdlog::logger& log, std::uint64_t sequence, const char* phase, const Frame& frame)
        {
            log.info("SSA frame shot={} phase={} frame={} timeMs={} valid={} generation={:016X} form={:08X} scoped={} contactKnown={} contact={} bipodMode={} latched={} aimValid={} aim=({:.6f},{:.6f},{:.6f})",
                sequence, phase, frame.index, frame.milliseconds, frame.valid, frame.generation, frame.form, frame.scoped,
                frame.contactKnown, frame.contact, frame.bipodMode, frame.bipodLatched, frame.aimValid, frame.aimPoint.x, frame.aimPoint.y, frame.aimPoint.z);
            logRay(log, sequence, phase, "scope", frame.scope);
            logRay(log, sequence, phase, "muzzle", frame.muzzle);
            log.info("SSA read shot={} phase={} scopeStage={} muzzleStage={}", sequence, phase,
                static_cast<unsigned>(frame.scopeReadStage), static_cast<unsigned>(frame.muzzleReadStage));
        }

        // Main thread owns start/stop. The worker only copies immutable value
        // samples; it never calls game APIs. All file I/O and log formatting
        // happen here. Polling is bounded to 50 ms and sleeps on this worker.
        struct Writer
        {
            std::jthread thread;
            void start()
            {
                if (thread.joinable()) return;
                writerFailed.store(false, std::memory_order_relaxed);
                thread = std::jthread([](std::stop_token stop) {
                    try {
                        const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_ScopeShots.log");
                        auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 4 * 1024 * 1024, 2, false);
                        spdlog::logger log("ROCK_ScopeShots", sink);
                        log.set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
                        log.set_error_handler([](const std::string&) { writerFailed.store(true, std::memory_order_release); });
                        log.info("SSA start version=1 pid={} build={} {} sampleMs=250 holdMs=8000 observational=true units=game direction=unit-vector angles=degrees guide=straight-launch-not-impact",
                            GetCurrentProcessId(), __DATE__, __TIME__);
                        log.flush();
                        std::uint64_t last{};
                        std::uint64_t statusAt{};
                        do {
                            const auto now = GetTickCount64();
                            if (enabled.load(std::memory_order_acquire) && now >= statusAt) {
                                Frame current{};
                                (void)presentedChannel.read(current);
                                log.info("SSA status enabled={} frame={} generation={:016X} form={:08X} playerSetups={} muzzleReads={} matchedLaunches={} samples={} skipped={} drops={} stageBits=1:form,2:muzzle,4:launch,8:scope,16:aim readStage=0:none,1:owner,2:node,3:world-bytes,4:finite-world",
                                    enabled.load(), current.index, current.generation, current.form, playerSetups.load(), muzzleReads.load(),
                                    matchedLaunches.load(), shotSequence.load(), skipped.load(), dropped.load());
                                log.flush();
                                statusAt = now + 5000;
                            }
                            Shot shot{};
                            if (completedChannel.read(shot) && shot.sequence > last) {
                                log.info("SSA shot={} skipped={} channelDrops={} sequenceGap={} timeMs={} epoch={} thread={} gameThread={} stage={} setupMatched={} form={:08X} equip={} handle={:08X} returned={} yawRad={:.9f} pitchRad={:.9f} setupToLaunchDeg={:.6f} scopeToLaunchDeg={:.6f} muzzleToLaunchDeg={:.6f} nativeAimToLaunchDeg={:.6f}",
                                    shot.sequence, skipped.load(), dropped.load(), shot.sequence - last - 1, shot.milliseconds,
                                    shot.epoch, shot.thread, shot.onGameThread, shot.stage, shot.setupMatched, shot.form, shot.equip, shot.handle,
                                    shot.launchReturned, shot.yaw, shot.pitch, policy::angleDegrees(shot.initial, shot.launch),
                                    policy::angleDegrees(shot.scopeAtSetup, shot.launch), policy::angleDegrees(shot.muzzleAtSetup, shot.launch),
                                    policy::angleDegrees(shot.nativeAim, shot.launch));
                                log.info("SSA comparison shot={} presentedScopeToLaunchDeg={:.6f} followingScopeToLaunchDeg={:.6f} scopeSetupToPresentedDeg={:.6f} previousFrameAgeMs={}",
                                    shot.sequence, policy::angleDegrees(shot.presented.scope, shot.launch), policy::angleDegrees(shot.following.scope, shot.launch),
                                    policy::angleDegrees(shot.scopeAtSetup, shot.presented.scope),
                                    shot.milliseconds >= shot.presented.milliseconds ? shot.milliseconds - shot.presented.milliseconds : 0);
                                logFrame(log, shot.sequence, "last-presented", shot.presented);
                                log.info("SSA read shot={} phase=origin-setup scopeStage={} muzzleStage={}", shot.sequence,
                                    static_cast<unsigned>(shot.scopeReadStage), static_cast<unsigned>(shot.muzzleReadStage));
                                logRay(log, shot.sequence, "origin-setup", "scope", shot.scopeAtSetup);
                                logRay(log, shot.sequence, "origin-setup", "muzzle", shot.muzzleAtSetup);
                                logRay(log, shot.sequence, "origin-setup", "native-aim", shot.nativeAim);
                                log.info("SSA aim shot={} valid={} target=({:.6f},{:.6f},{:.6f})", shot.sequence,
                                    shot.aimValid, shot.aimPoint.x, shot.aimPoint.y, shot.aimPoint.z);
                                logRay(log, shot.sequence, "origin-setup", "initial-launch", shot.initial);
                                logRay(log, shot.sequence, "launch", "launch", shot.launch);
                                logFrame(log, shot.sequence, "following-presentation", shot.following);
                                log.flush();
                                last = shot.sequence;
                            }
                            if (stop.stop_requested() || writerFailed.load(std::memory_order_acquire)) break;
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        } while (true);
                        log.info("SSA end pid={} skipped={} channelDrops={}", GetCurrentProcessId(), skipped.load(), dropped.load());
                        log.flush();
                    } catch (...) { writerFailed.store(true, std::memory_order_release); }
                });
            }
        };
        Writer& writer() { static Writer value; return value; }

        bool verifyCall(std::uintptr_t rva, std::uintptr_t target, const std::uint8_t* prefix, std::size_t prefixSize)
        {
            std::array<std::uint8_t, 16> bytes{};
            const auto site = REL::Offset(rva).address();
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(site - prefixSize), bytes.data(), prefixSize + 5) ||
                std::memcmp(bytes.data(), prefix, prefixSize) != 0 || bytes[prefixSize] != 0xE8) return false;
            std::int32_t displacement{};
            std::memcpy(&displacement, bytes.data() + prefixSize + 1, sizeof(displacement));
            return site + 5 + displacement == REL::Offset(target).address();
        }
    }

    bool install() noexcept
    {
        if (installed) return true;
        try {
            // Raw-disassembly witnesses: Fire at 0x140333908 and 0x140333C6C;
            // SetOriginAndDirection at 0x14104F420 calls Actor::GetFireNode.
            // ABI setup is verified as well as every original CALL target.
            constexpr std::uint8_t setupPrefix[]{ 0x48, 0x8D, 0x4D, 0xE0, 0x48, 0x89, 0x45, 0x58 };
            constexpr std::uint8_t launchPrefix[]{ 0x48, 0x8D, 0x55, 0xE0, 0x48, 0x8D, 0x4D, 0xD4 };
            constexpr std::uint8_t nodePrefix[]{ 0x8B, 0x57, 0x48 }; // followed by MOV RCX,[player singleton]
            std::array<std::uint8_t, 10> nodeSetup{};
            const auto nodeSite = REL::Offset(0x104F420).address();
            bool nodeAbi = native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(nodeSite - 10), nodeSetup.data(), nodeSetup.size()) &&
                std::memcmp(nodeSetup.data(), nodePrefix, 3) == 0 && nodeSetup[3] == 0x48 && nodeSetup[4] == 0x8B && nodeSetup[5] == 0x0D;
            std::int32_t playerDisplacement{};
            std::memcpy(&playerDisplacement, nodeSetup.data() + 6, 4);
            nodeAbi = nodeAbi && nodeSite + playerDisplacement == REL::Offset(0x5B043F0).address();
            if (!verifyCall(0x333908, 0x104F3A0, setupPrefix, sizeof(setupPrefix)) ||
                !verifyCall(0x333C6C, 0x104E860, launchPrefix, sizeof(launchPrefix)) || !nodeAbi ||
                !verifyCall(0x104F420, 0xDE85F0, nodeSetup.data(), nodeSetup.size()) || F4SE::GetTrampoline().free_size() < 48) {
                logger::error("ROCK: Scope shot diagnostics refused: native firing call bytes/ABI or trampoline space differ; no hooks installed.");
                return false;
            }
            originalSetOrigin = reinterpret_cast<SetOrigin>(REL::Offset(0x104F3A0).address());
            originalFireNode = reinterpret_cast<FireNode>(REL::Offset(0xDE85F0).address());
            originalLaunch = reinterpret_cast<Launch>(REL::Offset(0x104E860).address());
            auto& trampoline = F4SE::GetTrampoline();
            trampoline.write_call<5>(REL::Offset(0x333908).address(), &onSetOrigin);
            trampoline.write_call<5>(REL::Offset(0x104F420).address(), &onFireNode);
            trampoline.write_call<5>(REL::Offset(0x333C6C).address(), &onLaunch);
            installed = true;
            logger::info("ROCK: Native scope shot observation hooks installed (origin setup, muzzle read, final launch).");
            return true;
        } catch (...) { return false; }
    }

    void beginFrame() noexcept
    {
        gameThread.store(GetCurrentThreadId(), std::memory_order_release);
        const bool requested = installed && g_rockConfig.rockDebugNativeScopeShotAlignment;
        const bool previous = enabled.exchange(requested, std::memory_order_acq_rel);
        if (previous != requested) {
            epoch.fetch_add(1, std::memory_order_acq_rel);
            clearPresentation();
        }
        if (!requested) return;
        try { writer().start(); }
        catch (...) { writerFailed.store(true, std::memory_order_release); }
        if (writerFailed.load(std::memory_order_acquire) && !writerErrorReported) {
            writerErrorReported = true;
            try { logger::error("ROCK: Scope shot file writer failed; visual capture remains available, log evidence is incomplete."); } catch (...) {}
        }
    }

    void clearPresentation() noexcept
    {
        liveFrame = {};
        displayedShot = {};
        (void)presentedChannel.store({});
    }

    void shutdown() noexcept
    {
        enabled.store(false, std::memory_order_release);
        epoch.fetch_add(1, std::memory_order_acq_rel);
        clearPresentation();
        auto& thread = writer().thread;
        if (thread.joinable()) { thread.request_stop(); thread.join(); }
        (void)nativeChannel.store({});
        (void)completedChannel.store({});
        consumedSequence = shotSequence.load(std::memory_order_acquire);
        writerErrorReported = false;
    }

    void publishPresentation(RE::NiNode* weapon, std::uint64_t generation, std::uint32_t form,
        bool contactKnown, bool contact, bool bipodLatched) noexcept
    {
        if (!enabled.load(std::memory_order_acquire)) return;
        const auto& runtime = runtime_state::currentFrame();
        Frame frame{};
        frame.index = runtime.frameIndex;
        frame.milliseconds = GetTickCount64();
        frame.generation = generation;
        frame.form = form;
        frame.epoch = epoch.load(std::memory_order_acquire);
        frame.scoped = runtime.localScopeMenuOpen;
        frame.contactKnown = contactKnown;
        frame.contact = contact;
        frame.bipodMode = g_rockConfig.rockBipodMode;
        frame.bipodLatched = bipodLatched;
        frame.valid = weapon && generation && form && runtime.weaponDrawn;
        if (frame.valid) {
            readSight(frame);
            auto* data = fo4vr::getEquippedWeaponData();
            std::uintptr_t vtable{};
            RE::NiAVObject* muzzle{};
            RE::NiTransform world{};
            if (data && native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(data), vtable) &&
                vtable == fo4vr::EquippedWeaponData_vtable.address()) {
                frame.muzzleReadStage = ReadStage::Owner;
                if (native_memory::tryReadValue(&data->fireNode, muzzle) && readWorld(muzzle, world, frame.muzzleReadStage))
                    frame.muzzle = axisRay(world, 1);
            }
        }
        if (!frame.valid || liveFrame.generation != generation || liveFrame.form != form) displayedShot = {};
        liveFrame = frame;
        (void)presentedChannel.store(frame);
        Shot shot{};
        if (nativeChannel.read(shot) && shot.sequence > consumedSequence) {
            consumedSequence = shot.sequence;
            if (shot.epoch != frame.epoch) return;
            displayedShot = {};
            shot.following = frame;
            if (!completedChannel.store(shot)) dropped.fetch_add(1, std::memory_order_relaxed);
            if (frame.valid && shot.epoch == frame.epoch && shot.form == form &&
                shot.presented.valid && shot.presented.generation == generation && shot.presented.form == form && shot.presented.epoch == frame.epoch &&
                policy::fresh(shot.milliseconds, shot.presented.milliseconds, 100) &&
                policy::fresh(frame.milliseconds, shot.milliseconds, policy::kDisplayMilliseconds)) displayedShot = shot;
        }
    }

    void appendOverlay(debug::BodyOverlayFrame& frame) noexcept
    {
        if (!g_rockConfig.rockDebugNativeScopeShotAlignment) return;
        frame.drawText = true;
        const auto text = [&](float x, float y, const char* label, const float* color) {
            if (frame.textCount == frame.textEntries.size()) return;
            auto& entry = frame.textEntries[frame.textCount++];
            entry.x = x - 310; entry.y = y - 480; entry.size = 2.0f;
            entry.centeredInEye = true;
            std::snprintf(entry.text, sizeof(entry.text), "%s", label);
            std::copy_n(color, 4, entry.color);
        };
        constexpr float green[]{ 0.1f, 1.0f, 0.2f, 1.0f }, cyan[]{ 0.05f, 0.9f, 1.0f, 1.0f };
        constexpr float yellow[]{ 1.0f, 0.9f, 0.05f, 1.0f }, magenta[]{ 1.0f, 0.1f, 0.9f, 1.0f };
        constexpr float white[]{ 1.0f, 1.0f, 1.0f, 1.0f };
        if (!installed) {
            text(24, 256, "SCOPE SHOT HOOKS UNAVAILABLE; SEE ROCK.LOG", magenta);
            return;
        }
        const bool shotValid = displayedShot.sequence && policy::fresh(GetTickCount64(), displayedShot.milliseconds, policy::kDisplayMilliseconds);
        const auto& state = shotValid ? displayedShot.presented : liveFrame;
        const Ray scope = shotValid ? state.scope : liveFrame.scope;
        const Ray muzzle = shotValid ? displayedShot.muzzleAtSetup : liveFrame.muzzle;
        const Ray aim = shotValid ? displayedShot.nativeAim :
            (liveFrame.aimValid && liveFrame.muzzle.valid ? policy::ray(liveFrame.muzzle.origin, policy::subtract(liveFrame.aimPoint, liveFrame.muzzle.origin)) : Ray{});
        const Ray launch = shotValid ? displayedShot.launch : Ray{};
        const std::array offsets{
            policy::angularOffset(scope, state.right, state.up, muzzle),
            policy::angularOffset(scope, state.right, state.up, aim),
            policy::angularOffset(scope, state.right, state.up, launch),
        };
        float plotRange = 0.25f;
        for (const auto& offset : offsets) {
            if (offset.valid) plotRange = std::max({ plotRange, std::abs(offset.rightDegrees), std::abs(offset.upDegrees) });
        }
        plotRange = std::ceil(plotRange * 4.0f) * 0.25f;
        char line[128]{};
        std::snprintf(line, sizeof(line), "SCOPE SHOTS %s  CONTACT %s  BIPOD %s", shotValid ? "LAST SAMPLE" : "LIVE",
            !state.contactKnown ? "?" : state.contact ? "YES" : "NO", state.bipodLatched ? "LATCHED" : state.bipodMode ? "ON" : "OFF");
        text(24, 190, line, white);
        text(24, 212, "S GREEN=SCOPE  M CYAN=MUZZLE", green);
        text(24, 234, "A YELLOW=NATIVE AIM  L MAGENTA=LAUNCH", magenta);
        std::snprintf(line, sizeof(line), "DEG S-L %.3f  M-L %.3f  A-L %.3f (-1=UNKNOWN)",
            policy::angleDegrees(scope, launch), policy::angleDegrees(muzzle, launch), policy::angleDegrees(aim, launch));
        text(24, 256, line, white);
        std::snprintf(line, sizeof(line), "ANGULAR PLOT +/-%.2f DEG (AUTO SCALE)", plotRange);
        text(24, 278, line, white);
        text(24, 300, "STRAIGHT RAYS; NO IMPACT PREDICTION", white);
        text(24, 322, writerFailed.load() ? "LOG FAILED: SEE ROCK.LOG" : "LOWER SCOPE TO INSPECT RAYS (8 SEC)", white);
        if (scope.valid) text(210, 505, "S", green);
        const auto plot = [&](const policy::AngularOffset& offset, const char* label, const float* color) {
            if (offset.valid) text(210 + offset.rightDegrees * 150 / plotRange,
                505 - offset.upDegrees * 150 / plotRange, label, color);
        };
        plot(offsets[0], "M", cyan); plot(offsets[1], "A", yellow); plot(offsets[2], "L", magenta);
        if (shotValid) {
            std::snprintf(line, sizeof(line), "SHOT %llu AGE %.1fS FRAME %llu -> %llu",
                static_cast<unsigned long long>(displayedShot.sequence), (GetTickCount64() - displayedShot.milliseconds) * 0.001,
                static_cast<unsigned long long>(state.index), static_cast<unsigned long long>(displayedShot.following.index));
            text(24, 680, line, white);
        }
        // The submitted stereo matrices are not the magnified mono camera.
        // Only the labeled screen-space comparison is drawn while scoped.
        if (liveFrame.scoped) return;
        const auto guide = [&](const Ray& r, debug::MarkerOverlayRole role) {
            if (!r.valid || frame.markerCount + 2 > frame.markerEntries.size()) return;
            frame.drawMarkers = true;
            frame.markerEntries[frame.markerCount++] = { role, ni(r.origin), ni(policy::end(r)), 1.0f, true, true };
            const auto endpoint = ni(policy::end(r));
            frame.markerEntries[frame.markerCount++] = { role, endpoint, endpoint, 3.0f, true, false };
        };
        guide(scope, debug::MarkerOverlayRole::NativeScopeShotSight);
        guide(muzzle, debug::MarkerOverlayRole::NativeScopeShotMuzzle);
        guide(aim, debug::MarkerOverlayRole::NativeScopeShotAim);
        guide(launch, debug::MarkerOverlayRole::NativeScopeShotLaunch);
    }
}
