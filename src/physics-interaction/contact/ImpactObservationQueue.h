#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rock::impact_observation
{
    /*
     * Fixed-capacity bounded MPMC queue based on per-cell sequence numbers.
     * The Havok callback performs no allocation, logging, engine traversal, or
     * blocking synchronization when it publishes an observation. The game
     * thread is the only consumer today, but the producer contract remains
     * correct if FO4VR invokes contact listeners from more than one worker.
     */
    template <class T, std::size_t Capacity>
    class Queue
    {
    public:
        static_assert(Capacity >= 2);
        static_assert((Capacity & (Capacity - 1)) == 0,
            "Impact observation queue capacity must be a power of two");
        static_assert(std::is_trivially_copyable_v<T>);

        Queue() noexcept
        {
            for (std::size_t i = 0; i < Capacity; ++i) {
                _cells[i].sequence.store(i, std::memory_order_relaxed);
            }
        }

        Queue(const Queue&) = delete;
        Queue& operator=(const Queue&) = delete;

        [[nodiscard]] bool tryPush(const T& value) noexcept
        {
            std::size_t position = _enqueuePosition.load(
                std::memory_order_relaxed);
            Cell* cell = nullptr;
            for (;;) {
                cell = &_cells[position & (Capacity - 1)];
                const auto sequence = cell->sequence.load(
                    std::memory_order_acquire);
                const auto difference = static_cast<std::intptr_t>(sequence) -
                                        static_cast<std::intptr_t>(position);
                if (difference == 0) {
                    if (_enqueuePosition.compare_exchange_weak(
                            position,
                            position + 1,
                            std::memory_order_relaxed,
                            std::memory_order_relaxed)) {
                        break;
                    }
                } else if (difference < 0) {
                    _dropped.fetch_add(1, std::memory_order_relaxed);
                    return false;
                } else {
                    position = _enqueuePosition.load(
                        std::memory_order_relaxed);
                }
            }

            cell->value = value;
            cell->sequence.store(position + 1, std::memory_order_release);
            return true;
        }

        [[nodiscard]] bool tryPop(T& outValue) noexcept
        {
            std::size_t position = _dequeuePosition.load(
                std::memory_order_relaxed);
            Cell* cell = nullptr;
            for (;;) {
                cell = &_cells[position & (Capacity - 1)];
                const auto sequence = cell->sequence.load(
                    std::memory_order_acquire);
                const auto difference = static_cast<std::intptr_t>(sequence) -
                                        static_cast<std::intptr_t>(position + 1);
                if (difference == 0) {
                    if (_dequeuePosition.compare_exchange_weak(
                            position,
                            position + 1,
                            std::memory_order_relaxed,
                            std::memory_order_relaxed)) {
                        break;
                    }
                } else if (difference < 0) {
                    return false;
                } else {
                    position = _dequeuePosition.load(
                        std::memory_order_relaxed);
                }
            }

            outValue = cell->value;
            cell->sequence.store(position + Capacity, std::memory_order_release);
            return true;
        }

        [[nodiscard]] std::uint64_t droppedCount() const noexcept
        {
            return _dropped.load(std::memory_order_acquire);
        }

    private:
        struct Cell
        {
            std::atomic<std::size_t> sequence{ 0 };
            T value{};
        };

        alignas(64) std::array<Cell, Capacity> _cells{};
        alignas(64) std::atomic<std::size_t> _enqueuePosition{ 0 };
        alignas(64) std::atomic<std::size_t> _dequeuePosition{ 0 };
        std::atomic<std::uint64_t> _dropped{ 0 };
    };
}
