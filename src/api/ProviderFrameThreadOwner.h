#pragma once

#include <atomic>
#include <cstdint>

namespace rock::provider
{
    class ProviderFrameThreadOwner
    {
    public:
        // Called only from FRIK FrameBegin, never from a graph observation or
        // an API caller. The owner remains fixed for the plugin lifetime.
        bool beginFrame(std::uint32_t thread) noexcept
        {
            if (!thread) return false;
            std::uint32_t expected = 0;
            return _thread.compare_exchange_strong(expected, thread, std::memory_order_acq_rel) ||
                expected == thread;
        }

        [[nodiscard]] bool allows(std::uint32_t thread) const noexcept
        {
            return thread != 0 && owner() == thread;
        }

        [[nodiscard]] std::uint32_t owner() const noexcept
        {
            return _thread.load(std::memory_order_acquire);
        }

    private:
        std::atomic<std::uint32_t> _thread{ 0 };
    };
}
