#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

namespace rock::decoration_mode
{
    inline constexpr int kButtonId = 32;
    inline constexpr std::size_t kMaxBodies = 64;

    struct SurfaceContact {
        std::uint32_t heldBodyId{0x7FFF'FFFFu};
        std::uint32_t otherBodyId{0x7FFF'FFFFu};
        bool recent{};
    };

    // Physics callbacks publish one coherent pair. The game frame ages it;
    // normal grab cleanup clears it. No native pointer crosses that boundary.
    class SurfaceContactState {
    public:
        void publish(std::uint32_t held, std::uint32_t other) noexcept {
            _pair.store((std::uint64_t{held}<<32)|other,std::memory_order_relaxed);
            _age.store(0,std::memory_order_release);
        }
        SurfaceContact read() const noexcept {
            if (_age.load(std::memory_order_acquire)>=5) return {};
            const auto pair=_pair.load(std::memory_order_acquire);
            return {static_cast<std::uint32_t>(pair>>32),static_cast<std::uint32_t>(pair),true};
        }
        void tick() noexcept {
            auto age=_age.load(std::memory_order_acquire);
            if (age<5) _age.compare_exchange_strong(age,age+1,std::memory_order_release);
        }
        void clear() noexcept { _age.store(5,std::memory_order_release); }
    private:
        std::atomic<std::uint64_t> _pair{};
        std::atomic<unsigned> _age{5};
    };

    // Validate a fresh scan at placement. The grab lifecycle's historical
    // completeness flag describes its cached prep and is not a placement veto.
    [[nodiscard]] constexpr bool completeBodyScan(bool finished, std::size_t bodies,
        std::size_t accepted, std::uint32_t skippedOrFailed) noexcept
    {
        return finished && bodies > 0 && bodies <= kMaxBodies && accepted == bodies && skippedOrFailed == 0;
    }

    // A two-handed hold is one reference. Two different objects are ambiguous.
    [[nodiscard]] constexpr std::uint32_t singleObject(std::uint32_t right, std::uint32_t left) noexcept
    {
        return right && left && right != left ? 0 : (right ? right : left);
    }

    struct ClickState
    {
        bool armed{};
        bool draining{};
        bool reserved{};
        std::uint32_t request{};

        void update(bool enabled, std::uint32_t candidate, bool available,
            bool held, bool pressed, std::uint32_t ageMilliseconds) noexcept
        {
            request = 0;
            reserved = false;
            if (!available || ageMilliseconds > 100 || (!enabled && !draining)) {
                armed = false;
                draining = false;
                return;
            }
            // Retain ownership through the release of an accepted click even
            // after the object is released or the setting is disabled.
            reserved = draining || (enabled && candidate != 0);
            if (!held) {
                if (armed && pressed && enabled && candidate && !draining) request = candidate;
                armed = true;
                draining = false;
                return;
            }
            if (armed && pressed && enabled && candidate && !draining) {
                request = candidate;
                draining = true;
            }
            armed = false;
        }
    };
}
