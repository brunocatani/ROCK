#pragma once

#include <atomic>
#include <cstdint>
#include <ROCK/Core.h>

namespace rock::provider
{
    // One game clock with two publication phases. The game thread advances it;
    // API/physics readers observe timestamps without accessing runtime objects.
    class ProviderFrameClock
    {
    public:
        std::uint64_t beginPhase(api::core::AnimationPhaseV1 phase, std::uint64_t gameFrame) noexcept
        {
            // Native graph observations can occur between Complete and BeforeRock.
            // BeforeRock repeats the measured frame established at FRIK FrameBegin;
            // native graph observations never advance the clock or retire leases.
            if (phase == api::core::AnimationPhaseV1::BeforeRock) beginFrame(gameFrame);
            return phase == api::core::AnimationPhaseV1::NativeGraphOutput ? gameFrame : current();
        }

        void beginFrame(std::uint64_t gameFrame) noexcept
        {
            _current.store(gameFrame, std::memory_order_release);
        }

        [[nodiscard]] std::uint64_t current() const noexcept
        {
            return _current.load(std::memory_order_acquire);
        }

        std::uint64_t publishFrame() noexcept
        {
            const auto frame = current();
            _published.store(frame, std::memory_order_release);
            return frame;
        }

        // Retire leases after the consuming update, preserving one-frame drives.
        [[nodiscard]] std::uint64_t leaseBoundary() const noexcept
        {
            return _published.load(std::memory_order_acquire);
        }

    private:
        std::atomic<std::uint64_t> _current{0};
        std::atomic<std::uint64_t> _published{0};
    };
}
