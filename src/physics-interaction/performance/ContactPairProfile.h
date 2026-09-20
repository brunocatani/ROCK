#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace rock::performance_profiler
{
    enum class ContactStage : std::uint8_t
    {
        SimulationInput, SimulationNative, SimulationKept, Manifold, Impulse,
        PlayerMeleeDropped, OtherMeleeDropped, PlayerMeleeForwarded, OtherMeleeForwarded, Count
    };

    inline constexpr std::uint32_t kUnknownContactDetail = 0xFFFF'FFFFu;
    struct ContactPair
    {
        // Opaque log identity only. Never converted back to an engine pointer.
        std::uintptr_t world = 0;
        std::uint32_t bodyA = kUnknownContactDetail, bodyB = kUnknownContactDetail;
        std::uint32_t shapeA = kUnknownContactDetail, shapeB = kUnknownContactDetail;
        std::uint32_t layerA = kUnknownContactDetail, layerB = kUnknownContactDetail;

        void canonicalize() noexcept
        {
            if (bodyB < bodyA) {
                std::swap(bodyA, bodyB);
                std::swap(shapeA, shapeB);
                std::swap(layerA, layerB);
            }
        }
    };

    struct ContactPairCounts
    {
        ContactPair pair{};
        std::array<std::uint64_t, static_cast<std::size_t>(ContactStage::Count)> counts{};
        std::uint64_t firstFrame = 0, lastFrame = 0;
        std::uint64_t events = 0;
        bool layerChanged = false;
    };

    // Physics workers only try a single bucket lease; they never wait, allocate,
    // format text, or retain engine objects. The frame thread drains into the
    // existing profiler writer. A busy bucket carries to the next window;
    // rejected writes and carried buckets are reported explicitly.
    template <std::size_t Capacity>
    class ContactPairTable
    {
        static constexpr std::size_t kWays = 4;
        static_assert(Capacity >= kWays && Capacity % kWays == 0);
        struct Bucket
        {
            std::atomic_flag busy = ATOMIC_FLAG_INIT;
            std::uint64_t generation = 0;
            std::array<ContactPairCounts, kWays> slots{};
        };

    public:
        struct Snapshot
        {
            std::array<ContactPairCounts, Capacity> entries{};
            std::size_t size = 0;
            std::uint64_t dropped = 0;
            std::uint32_t carriedBuckets = 0;
        };

        bool record(ContactPair pair, ContactStage stage, std::uint64_t frame, std::uint64_t generation) noexcept
        {
            const auto stageIndex = static_cast<std::size_t>(stage);
            if (!pair.world || pair.bodyA == pair.bodyB || pair.bodyA >= 0x7FFF'FFFFu ||
                pair.bodyB >= 0x7FFF'FFFFu || stageIndex >= static_cast<std::size_t>(ContactStage::Count)) return false;
            pair.canonicalize();
            std::uint64_t hash = (static_cast<std::uint64_t>(pair.bodyA) << 32) | pair.bodyB;
            hash ^= pair.world;
            hash ^= (static_cast<std::uint64_t>(pair.shapeA) << 32) | pair.shapeB;
            hash ^= hash >> 30; hash *= 0xBF58476D1CE4E5B9ull;
            hash ^= hash >> 27; hash *= 0x94D049BB133111EBull;
            hash ^= hash >> 31;
            auto& bucket = _buckets[hash % _buckets.size()];
            if (bucket.busy.test_and_set(std::memory_order_acquire)) {
                _dropped.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            if (bucket.generation > generation) {
                bucket.busy.clear(std::memory_order_release);
                return false; // In-flight observation from before a settings reset.
            }
            if (bucket.generation != generation) {
                bucket.slots = {};
                bucket.generation = generation;
            }
            ContactPairCounts* found = nullptr;
            ContactPairCounts* empty = nullptr;
            for (auto& slot : bucket.slots) {
                if (!slot.events) { if (!empty) empty = &slot; continue; }
                if (slot.pair.world == pair.world && slot.pair.bodyA == pair.bodyA && slot.pair.bodyB == pair.bodyB &&
                    slot.pair.shapeA == pair.shapeA && slot.pair.shapeB == pair.shapeB) { found = &slot; break; }
            }
            if (!found) found = empty;
            if (found) {
                if (!found->events) { found->pair = pair; found->firstFrame = frame; found->lastFrame = frame; }
                const auto mergeLayer = [&](std::uint32_t& previous, std::uint32_t current) {
                    if (current == kUnknownContactDetail) return;
                    found->layerChanged |= previous != kUnknownContactDetail && previous != current;
                    previous = current;
                };
                mergeLayer(found->pair.layerA, pair.layerA);
                mergeLayer(found->pair.layerB, pair.layerB);
                if (frame < found->firstFrame) found->firstFrame = frame;
                if (frame > found->lastFrame) found->lastFrame = frame;
                ++found->counts[stageIndex];
                ++found->events;
            }
            bucket.busy.clear(std::memory_order_release);
            if (!found) _dropped.fetch_add(1, std::memory_order_relaxed);
            return found != nullptr;
        }

        Snapshot take(std::uint64_t generation) noexcept
        {
            Snapshot result;
            for (auto& bucket : _buckets) {
                if (bucket.busy.test_and_set(std::memory_order_acquire)) { ++result.carriedBuckets; continue; }
                if (bucket.generation == generation) {
                    for (const auto& slot : bucket.slots) if (slot.events) result.entries[result.size++] = slot;
                }
                if (bucket.generation <= generation) { bucket.slots = {}; bucket.generation = generation; }
                bucket.busy.clear(std::memory_order_release);
            }
            result.dropped = _dropped.exchange(0, std::memory_order_relaxed);
            return result;
        }

    private:
        std::array<Bucket, Capacity / kWays> _buckets{};
        std::atomic<std::uint64_t> _dropped{0};
    };
}
