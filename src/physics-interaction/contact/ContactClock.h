#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

namespace rock::contact_clock
{
    /*
     * The one clock every contact store stamps against: hand semantic
     * contacts, dynamic hand surface contacts, body contacts and touch.
     * Physics callbacks stamp with the current values; game-frame readers
     * compute age from them, so every store measures age the same way.
     *
     * The frame counter gives run continuity and publication identity. The
     * seconds value accumulates only measured game time, so freshness windows
     * are rate-independent and an unmeasurable frame holds them instead of
     * expiring them on fabricated time. The clock is monotonic for the
     * process; stores invalidate their own records and the clock never
     * rewinds under them.
     */
    class ContactClock
    {
    public:
        // Game thread, once per frame, after the frame's contacts are resolved.
        void advance(float validDeltaSeconds) noexcept
        {
            _frame.fetch_add(1, std::memory_order_acq_rel);
            if (std::isfinite(validDeltaSeconds) && validDeltaSeconds > 0.0f) {
                _seconds.store(_seconds.load(std::memory_order_relaxed) + validDeltaSeconds, std::memory_order_release);
            }
        }

        [[nodiscard]] std::uint32_t frame() const noexcept { return _frame.load(std::memory_order_acquire); }
        [[nodiscard]] double seconds() const noexcept { return _seconds.load(std::memory_order_acquire); }

    private:
        std::atomic<std::uint32_t> _frame{ 0 };
        std::atomic<double> _seconds{ 0.0 };
    };
}
