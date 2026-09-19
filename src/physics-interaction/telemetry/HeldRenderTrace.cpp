#include "physics-interaction/telemetry/HeldRenderTrace.h"

#include "physics-interaction/telemetry/DynamicColliderTrace.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"
#include "RE/NetImmerse/NiNode.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <Windows.h>

namespace rock::held_render_trace
{
    namespace
    {
        using Worker = void (*)(const float*, std::uint8_t, float*, const float*);
        using OpaqueWorker = void (*)(void*, void*, void*);
        // Verified against FO4VR raw disassembly: wrapper 0x1D14A60 tail-jumps
        // here; shader callers at 0x28B6D5E and 0x28CFA33 pass geometry.world
        // (+0x70). RCX=input NiTransform, DL=mode, R8=16-float output, R9=origin.
        // The stolen 14 bytes are complete position-independent instructions.
        constexpr std::uintptr_t kWorkerRva = 0x1D14CC0;
        constexpr std::array<std::uint8_t, 14> kPrefix{
            0x48, 0x8B, 0xC4, 0x53, 0x48, 0x81, 0xEC, 0xE0, 0, 0, 0, 0x49, 0x8B, 0xD8 };
        // BSDFPrePassShader ctor 0x2878930 installs vtable 0x30B8C68;
        // slot 9 is this geometry setup. Raw 0x287CF85 and 0x28B6B97
        // both read the live pass's geometry at RDX+0x18. R8 is shader
        // technique data, with flags at +0x40 in both implementations.
        constexpr std::uintptr_t kOpaqueRva = 0x287CF60;
        constexpr std::array<std::uint8_t, 14> kOpaquePrefix{
            0x48, 0x8B, 0xC4, 0x48, 0x89, 0x50, 0x10, 0x48, 0x89, 0x48, 0x08, 0x55, 0x53, 0x56 };
        constexpr std::size_t kShapesPerHand = 4;
        constexpr std::size_t kCapacity = 128;
        static_assert(sizeof(RE::NiTransform) == 16 * sizeof(float));

        struct Targets
        {
            std::atomic<std::uint64_t> revision{ 0 }, trace{ 0 };
            std::array<std::atomic<std::uintptr_t>, kShapesPerHand> addresses{};
            std::array<std::array<std::atomic<std::uint64_t>, kShapesPerHand>, 2> lastPhase{};
        };
        struct Sample
        {
            std::uint64_t epoch{}, phase{}, trace{}, ticks{};
            std::uint32_t thread{}, hand{}, shape{}, mode{}, valid{}, path{};
            RE::NiTransform input{}, inputAfter{}, previous{};
            std::array<float, 16> output{};
            std::array<float, 4> origin{}, originRight{};
            std::uint64_t shaderFlags{};
        };
        // Registrations contain comparison tokens only, never dereferenced by
        // the renderer. All copied pointers belong to the live native call.
        std::array<Targets, 2> targets;
        std::atomic<std::uint64_t> activeEpoch{ 0 }, currentPhase{ 0 }, dropped{ 0 };
        std::uint64_t nextEpoch = 0; // game thread only
        Worker original = nullptr; // installed once, lives for the process
        OpaqueWorker originalOpaque = nullptr;
        std::atomic<std::uint64_t> failedPassReads{ 0 };
        std::atomic<std::uint64_t> opaqueCalls{ 0 }, opaqueMatches{ 0 };
        bool attempted = false;
        std::atomic_flag queueGate = ATOMIC_FLAG_INIT;
        std::array<Sample, kCapacity> queue;
        std::size_t count = 0;

        // Renderer producers and the game-thread consumer never wait. A busy
        // or full diagnostic buffer drops the sample and reports the count.
        void enqueue(const Sample& sample) noexcept
        {
            if (queueGate.test_and_set(std::memory_order_acquire)) {
                dropped.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            if (count < queue.size()) queue[count++] = sample;
            else dropped.fetch_add(1, std::memory_order_relaxed);
            queueGate.clear(std::memory_order_release);
        }

        bool beginSample(std::uintptr_t worldAddress, std::uint32_t path, Sample& sample) noexcept
        {
            const auto epoch = activeEpoch.load(std::memory_order_acquire);
            const auto phase = currentPhase.load(std::memory_order_acquire);
            if (epoch && sampleFrame(phase >> 2) && worldAddress) {
                for (std::size_t hand = 0; hand < targets.size(); ++hand) {
                    auto& target = targets[hand];
                    const auto revision = target.revision.load(std::memory_order_acquire);
                    if (revision & 1u) continue;
                    const auto trace = target.trace.load(std::memory_order_relaxed);
                    if (!trace) continue;
                    for (std::size_t shape = 0; shape < kShapesPerHand; ++shape) {
                        if (target.addresses[shape].load(std::memory_order_relaxed) != worldAddress) continue;
                        if (revision != target.revision.load(std::memory_order_acquire)) break;
                        if (target.lastPhase[path][shape].exchange(phase, std::memory_order_relaxed) == phase) return false;
                        sample.epoch = epoch;
                        sample.phase = phase;
                        sample.trace = trace;
                        sample.hand = static_cast<std::uint32_t>(hand);
                        sample.shape = static_cast<std::uint32_t>(shape);
                        sample.path = path;
                        sample.thread = GetCurrentThreadId();
                        LARGE_INTEGER stamp{};
                        QueryPerformanceCounter(&stamp);
                        sample.ticks = static_cast<std::uint64_t>(stamp.QuadPart);
                        return true;
                    }
                }
            }
            return false;
        }

        void hook(const float* input, std::uint8_t mode, float* output, const float* origin) noexcept
        {
            Sample sample{};
            const bool matched = beginSample(reinterpret_cast<std::uintptr_t>(input), 0, sample);
            if (matched) {
                sample.mode = mode;
                sample.valid = native_memory::guardedCopyFromMemory(input, &sample.input, sizeof(sample.input)) ? 1u : 0u;
                if (origin && native_memory::guardedCopyFromMemory(origin, sample.origin.data(), sizeof(sample.origin))) sample.valid |= 2u;
            }
            original(input, mode, output, origin);
            if (!matched) return;
            if (native_memory::guardedCopyFromMemory(input, &sample.inputAfter, sizeof(sample.inputAfter))) sample.valid |= 4u;
            if (output && native_memory::guardedCopyFromMemory(output, sample.output.data(), sizeof(sample.output))) sample.valid |= 8u;
            enqueue(sample);
        }

        bool readPassGeometry(const void* pass, RE::NiAVObject*& geometry) noexcept
        {
            if (reinterpret_cast<std::uintptr_t>(pass) < 0x10000) return false;
            // This pointer is borrowed from the native call, which itself reads
            // this exact field. SEH avoids a VirtualQuery on every scene draw;
            // deeper, guarded copies occur only after a registered shape matches.
            __try {
                geometry = *reinterpret_cast<RE::NiAVObject* const*>(static_cast<const char*>(pass) + 0x18);
                return reinterpret_cast<std::uintptr_t>(geometry) >= 0x10000 &&
                    (reinterpret_cast<std::uintptr_t>(geometry) & 0xF) == 0;
            } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
        }

        void opaqueHook(void* shader, void* pass, void* technique) noexcept
        {
            Sample sample{};
            RE::NiAVObject* geometry = nullptr;
            bool matched = false;
            if (activeEpoch.load(std::memory_order_acquire) && sampleFrame(currentPhase.load(std::memory_order_acquire) >> 2) &&
                (targets[0].trace.load(std::memory_order_relaxed) || targets[1].trace.load(std::memory_order_relaxed))) {
                opaqueCalls.fetch_add(1, std::memory_order_relaxed);
                if (!readPassGeometry(pass, geometry)) failedPassReads.fetch_add(1, std::memory_order_relaxed);
                else matched = beginSample(reinterpret_cast<std::uintptr_t>(&geometry->world), 1, sample);
            }
            if (matched) {
                opaqueMatches.fetch_add(1, std::memory_order_relaxed);
                if (native_memory::tryReadValue(&geometry->world, sample.input)) sample.valid |= 1u;
                if (native_memory::tryReadValue(&geometry->previousWorld, sample.previous)) sample.valid |= 16u;
                if (native_memory::tryReadField(technique, 0x40, sample.mode)) sample.valid |= 32u;
                // Both native geometry setup paths read property at +0x178,
                // then its flags at +0x30. No retained renderer pointers.
                void* property = nullptr;
                if (native_memory::tryReadField(geometry, 0x178, property) && property &&
                    native_memory::tryReadField(property, 0x30, sample.shaderFlags)) sample.valid |= 64u;
                // Raw 0x1D14A60 and 0x287E4BD/0x287E51A agree on these
                // renderer-selected eye origins. Capture both, without calling
                // native camera setup or changing its current renderer context.
                void* renderer = nullptr;
                const auto rendererSlot = REL::Offset(0x6235AC8).address();
                const auto defaultSlot = REL::Offset(0x6235AC0).address();
                if (native_memory::tryReadValue(reinterpret_cast<void* const*>(rendererSlot), renderer) &&
                    (renderer || native_memory::tryReadValue(reinterpret_cast<void* const*>(defaultSlot), renderer)) && renderer) {
                    if (native_memory::tryReadField(renderer, 0x2590, sample.origin)) sample.valid |= 2u;
                    if (native_memory::tryReadField(renderer, 0x25A0, sample.originRight)) sample.valid |= 128u;
                }
            }
            originalOpaque(shader, pass, technique);
            if (!matched) return;
            if (native_memory::tryReadValue(&geometry->world, sample.inputAfter)) sample.valid |= 4u;
            enqueue(sample);
        }
    }

    void install()
    {
        if (!g_rockConfig.rockDebugGrabFrameLogging && !g_rockConfig.rockDebugShowSkeletonBoneVisualizer &&
            !g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers) return;
        if (!attempted) {
            attempted = true;
            void* trampoline = nullptr;
            if (entry_trampoline_hook::install("held renderer witness", kWorkerRva, kPrefix.data(), kPrefix.size(),
                    reinterpret_cast<void*>(&hook), trampoline)) original = reinterpret_cast<Worker>(trampoline);
            trampoline = nullptr;
            if (entry_trampoline_hook::install("held opaque renderer witness", kOpaqueRva, kOpaquePrefix.data(), kOpaquePrefix.size(),
                    reinterpret_cast<void*>(&opaqueHook), trampoline)) originalOpaque = reinterpret_cast<OpaqueWorker>(trampoline);
        }
        if (!original) {
            ROCK_LOG_ERROR(Hand, "Held renderer witness unavailable: native worker entry did not pass installation checks");
        }
        if (!originalOpaque) ROCK_LOG_ERROR(Hand, "Held opaque renderer witness unavailable: native entry did not pass installation checks");
    }

    void initialize()
    {
        if ((!original && !originalOpaque) || !dynamic_collider_trace::presentationEnabled()) return;
        LARGE_INTEGER frequency{};
        QueryPerformanceFrequency(&frequency);
        activeEpoch.store(++nextEpoch, std::memory_order_release);
        dynamic_collider_trace::write("RENDER_HELD start version=3 epoch={} qpcFrequency={} workerRva={:X} opaqueRva={:X} worker={} opaque={} shapesPerHand={} buffer={} burst=12/120 solverWitness=true observational=true",
            nextEpoch, frequency.QuadPart, kWorkerRva, kOpaqueRva, original != nullptr, originalOpaque != nullptr, kShapesPerHand, kCapacity);
    }

    std::uint64_t sampledFrame() noexcept
    {
        if (!activeEpoch.load(std::memory_order_acquire)) return 0;
        const auto frame = currentPhase.load(std::memory_order_acquire) >> 2;
        return sampleFrame(frame) ? frame : 0;
    }

    void clearHand(bool isLeft) noexcept
    {
        auto& target = targets[isLeft ? 1u : 0u];
        target.revision.fetch_add(1, std::memory_order_acq_rel);
        target.trace.store(0, std::memory_order_relaxed);
        for (std::size_t i = 0; i < kShapesPerHand; ++i) {
            target.addresses[i].store(0, std::memory_order_relaxed);
            for (auto& path : target.lastPhase) path[i].store(0, std::memory_order_relaxed);
        }
        target.revision.fetch_add(1, std::memory_order_release);
    }

    void shutdown() noexcept
    {
        activeEpoch.store(0, std::memory_order_release);
        clearHand(false);
        clearHand(true);
        currentPhase.store(0, std::memory_order_release);
    }

    void registerRoot(bool isLeft, std::uint64_t trace, RE::NiAVObject* root)
    {
        clearHand(isLeft);
        if (!root || !trace || !activeEpoch.load(std::memory_order_acquire)) return;
        std::array<std::uintptr_t, kShapesPerHand> addresses{};
        std::size_t found = 0;
        const auto traversal = weapon_scene::visitScene(root, [&](RE::NiAVObject* node) {
            if (node->IsTriShape()) {
                addresses[found] = reinterpret_cast<std::uintptr_t>(&node->world);
                dynamic_collider_trace::write("RENDER_HELD register trace={} hand={} shape={} name='{}' worldAddress={:X}",
                    trace, isLeft ? "left" : "right", found, node->name.c_str(), addresses[found]);
                ++found;
            }
            return found < addresses.size();
        });
        auto& target = targets[isLeft ? 1u : 0u];
        target.revision.fetch_add(1, std::memory_order_acq_rel);
        for (std::size_t i = 0; i < addresses.size(); ++i) target.addresses[i].store(addresses[i], std::memory_order_relaxed);
        target.trace.store(trace, std::memory_order_relaxed);
        target.revision.fetch_add(1, std::memory_order_release);
        dynamic_collider_trace::write("RENDER_HELD registered trace={} hand={} shapes={} visited={}", trace, isLeft ? "left" : "right", found, traversal.visited);
    }

    void recordPhase(Phase phase, std::uint64_t frame)
    {
        const auto epoch = activeEpoch.load(std::memory_order_acquire);
        if (!epoch || !dynamic_collider_trace::presentationEnabled()) return;
        currentPhase.store((frame << 2) | static_cast<std::uint64_t>(phase), std::memory_order_release);
        std::array<Sample, kCapacity> batch;
        std::size_t available = 0;
        if (!queueGate.test_and_set(std::memory_order_acquire)) {
            available = count;
            for (std::size_t i = 0; i < available; ++i) batch[i] = queue[i];
            count = 0;
            queueGate.clear(std::memory_order_release);
        }
        for (std::size_t i = 0; i < available; ++i) {
            const auto& s = batch[i];
            if (s.epoch != epoch) continue;
            const auto& p = s.input.translate;
            const auto& r = s.input.rotate.entry;
            dynamic_collider_trace::write(
                "RENDER_HELD sample frame={} phase={} trace={} hand={} shape={} mode={} valid={} ticks={} thread={} T=({:.4f},{:.4f},{:.4f}) S={:.6f} R=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f}) origin=({:.4f},{:.4f},{:.4f}) duringCall={:.5f}gu/{:.5f}deg outputT=({:.4f},{:.4f},{:.4f},{:.4f}) path={}",
                s.phase >> 2, s.phase & 3u, s.trace, s.hand ? "left" : "right", s.shape, s.mode, s.valid, s.ticks, s.thread,
                p.x, p.y, p.z, s.input.scale, r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
                s.origin[0], s.origin[1], s.origin[2],
                (s.valid & 5u) == 5u ? tracked_hand_isolation_policy::translationGameUnits(s.input, s.inputAfter) : -1.0f,
                (s.valid & 5u) == 5u ? tracked_hand_isolation_policy::rotationDegrees(s.input, s.inputAfter) : -1.0f,
                s.output[12], s.output[13], s.output[14], s.output[15], s.path ? "opaque" : "transform-helper");
            if (s.path == 1) {
                dynamic_collider_trace::write(
                    "RENDER_HELD opaque-detail frame={} phase={} trace={} hand={} shape={} valid={} techniqueFlags={:X} shaderFlags={:X} previousT=({:.4f},{:.4f},{:.4f}) rightOrigin=({:.4f},{:.4f},{:.4f})",
                    s.phase >> 2, s.phase & 3u, s.trace, s.hand ? "left" : "right", s.shape, s.valid, s.mode, s.shaderFlags,
                    s.previous.translate.x, s.previous.translate.y, s.previous.translate.z, s.originRight[0], s.originRight[1], s.originRight[2]);
            }
        }
        if (frame % 300 == 0 && phase == Phase::AfterWorldFinal) {
            dynamic_collider_trace::write("RENDER_HELD status frame={} dropped={} failedPassReads={} opaqueCalls={} opaqueMatches={}", frame,
                dropped.load(std::memory_order_relaxed), failedPassReads.load(std::memory_order_relaxed),
                opaqueCalls.load(std::memory_order_relaxed), opaqueMatches.load(std::memory_order_relaxed));
        }
    }
}
