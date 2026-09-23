#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace rock::world_weapon_material_visibility
{
    struct AttachmentEvent
    {
        std::uint32_t referenceID = 0;
        bool attached = false;
    };

    // The native attach/detach source serializes its callbacks under one lock
    // (140443240). That is one logical producer, possibly on loading threads;
    // the game-frame consumer alone owns references and scene mutations.
    class PendingEvents
    {
    public:
        static constexpr std::size_t kCapacity = 2048;

        bool push(AttachmentEvent event) noexcept
        {
            const auto write = _write.load(std::memory_order_relaxed);
            const auto next = (write + 1) % kCapacity;
            if (next == _read.load(std::memory_order_acquire)) return false;
            _events[write] = event;
            _write.store(next, std::memory_order_release);
            return true;
        }

        bool pop(AttachmentEvent& event) noexcept
        {
            const auto read = _read.load(std::memory_order_relaxed);
            if (read == _write.load(std::memory_order_acquire)) return false;
            event = _events[read];
            _read.store((read + 1) % kCapacity, std::memory_order_release);
            return true;
        }

        void discard() noexcept
        {
            // Consumer operation; never reset a concurrently used producer index.
            AttachmentEvent event;
            for (std::size_t i = 0; i < kCapacity && pop(event); ++i) {}
        }

    private:
        std::array<AttachmentEvent, kCapacity> _events{};
        std::atomic<std::size_t> _read{ 0 }, _write{ 0 };
    };
}
