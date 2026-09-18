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
        // Verified against FO4VR raw disassembly: wrapper 0x1D14A60 tail-jumps
        // here; shader callers at 0x28B6D5E and 0x28CFA33 pass geometry.world
        // (+0x70). RCX=input NiTransform, DL=mode, R8=16-float output, R9=origin.
        // The stolen 14 bytes are complete position-independent instructions.
        constexpr std::uintptr_t kWorkerRva = 0x1D14CC0;
        constexpr std::array<std::uint8_t, 14> kPrefix{
            0x48, 0x8B, 0xC4, 0x53, 0x48, 0x81, 0xEC, 0xE0, 0, 0, 0, 0x49, 0x8B, 0xD8 };
        constexpr std::size_t kShapesPerHand = 4;
        constexpr std::size_t kCapacity = 128;
        static_assert(sizeof(RE::NiTransform) == 16 * sizeof(float));

        struct Targets
        {
            std::atomic<std::uint64_t> revision{ 0 }, trace{ 0 };
            std::array<std::atomic<std::uintptr_t>, kShapesPerHand> addresses{};
            std::array<std::atomic<std::uint64_t>, kShapesPerHand> lastPhase{};
        };
        struct Sample
        {
            std::uint64_t epoch{}, phase{}, trace{}, ticks{};
            std::uint32_t thread{}, hand{}, shape{}, mode{}, valid{};
            RE::NiTransform input{}, inputAfter{};
            std::array<float, 16> output{};
            std::array<float, 4> origin{};
        };
        // Registrations contain comparison tokens only, never dereferenced by
        // the renderer. All copied pointers belong to the live native call.
        std::array<Targets, 2> targets;
        std::atomic<std::uint64_t> activeEpoch{ 0 }, currentPhase{ 0 }, dropped{ 0 };
        std::uint64_t nextEpoch = 0; // game thread only
        Worker original = nullptr; // installed once, lives for the process
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

        void hook(const float* input, std::uint8_t mode, float* output, const float* origin) noexcept
        {
            const auto epoch = activeEpoch.load(std::memory_order_acquire);
            const auto phase = currentPhase.load(std::memory_order_acquire);
            Sample sample{};
            bool matched = false;
            if (epoch && phase && input) {
                for (std::size_t hand = 0; hand < targets.size() && !matched; ++hand) {
                    auto& target = targets[hand];
                    const auto revision = target.revision.load(std::memory_order_acquire);
                    if (revision & 1u) continue;
                    const auto trace = target.trace.load(std::memory_order_relaxed);
                    if (!trace) continue;
                    for (std::size_t shape = 0; shape < kShapesPerHand; ++shape) {
                        if (target.addresses[shape].load(std::memory_order_relaxed) != reinterpret_cast<std::uintptr_t>(input)) continue;
                        if (revision != target.revision.load(std::memory_order_acquire)) break;
                        if (target.lastPhase[shape].exchange(phase, std::memory_order_relaxed) == phase) break;
                        sample.epoch = epoch;
                        sample.phase = phase;
                        sample.trace = trace;
                        sample.hand = static_cast<std::uint32_t>(hand);
                        sample.shape = static_cast<std::uint32_t>(shape);
                        sample.mode = mode;
                        sample.thread = GetCurrentThreadId();
                        LARGE_INTEGER stamp{};
                        QueryPerformanceCounter(&stamp);
                        sample.ticks = static_cast<std::uint64_t>(stamp.QuadPart);
                        sample.valid = native_memory::guardedCopyFromMemory(input, &sample.input, sizeof(sample.input)) ? 1u : 0u;
                        if (origin && native_memory::guardedCopyFromMemory(origin, sample.origin.data(), sizeof(sample.origin))) sample.valid |= 2u;
                        matched = true;
                        break;
                    }
                }
            }
            original(input, mode, output, origin);
            if (!matched) return;
            if (native_memory::guardedCopyFromMemory(input, &sample.inputAfter, sizeof(sample.inputAfter))) sample.valid |= 4u;
            if (output && native_memory::guardedCopyFromMemory(output, sample.output.data(), sizeof(sample.output))) sample.valid |= 8u;
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
        }
        if (!original) {
            ROCK_LOG_ERROR(Hand, "Held renderer witness unavailable: native worker entry did not pass installation checks");
            return;
        }
    }

    void initialize()
    {
        if (!original || !dynamic_collider_trace::presentationEnabled()) return;
        LARGE_INTEGER frequency{};
        QueryPerformanceFrequency(&frequency);
        activeEpoch.store(++nextEpoch, std::memory_order_release);
        dynamic_collider_trace::write("RENDER_HELD start epoch={} qpcFrequency={} workerRva={:X} shapesPerHand={} buffer={} observational=true",
            nextEpoch, frequency.QuadPart, kWorkerRva, kShapesPerHand, kCapacity);
    }

    void clearHand(bool isLeft) noexcept
    {
        auto& target = targets[isLeft ? 1u : 0u];
        target.revision.fetch_add(1, std::memory_order_acq_rel);
        target.trace.store(0, std::memory_order_relaxed);
        for (std::size_t i = 0; i < kShapesPerHand; ++i) {
            target.addresses[i].store(0, std::memory_order_relaxed);
            target.lastPhase[i].store(0, std::memory_order_relaxed);
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
                "RENDER_HELD sample frame={} phase={} trace={} hand={} shape={} mode={} valid={} ticks={} thread={} T=({:.4f},{:.4f},{:.4f}) S={:.6f} R=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f}) origin=({:.4f},{:.4f},{:.4f}) duringCall={:.5f}gu/{:.5f}deg outputT=({:.4f},{:.4f},{:.4f},{:.4f})",
                s.phase >> 2, s.phase & 3u, s.trace, s.hand ? "left" : "right", s.shape, s.mode, s.valid, s.ticks, s.thread,
                p.x, p.y, p.z, s.input.scale, r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
                s.origin[0], s.origin[1], s.origin[2],
                (s.valid & 5u) == 5u ? tracked_hand_isolation_policy::translationGameUnits(s.input, s.inputAfter) : -1.0f,
                (s.valid & 5u) == 5u ? tracked_hand_isolation_policy::rotationDegrees(s.input, s.inputAfter) : -1.0f,
                s.output[12], s.output[13], s.output[14], s.output[15]);
        }
        if (frame % 300 == 0 && phase == Phase::AfterWorldFinal) {
            dynamic_collider_trace::write("RENDER_HELD status frame={} dropped={}", frame, dropped.load(std::memory_order_relaxed));
        }
    }
}
