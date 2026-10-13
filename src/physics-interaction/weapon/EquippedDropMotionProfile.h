#pragma once

#include <cstdint>

namespace rock::equipped_drop_motion_profile
{
    inline constexpr std::uint16_t kInvalid = 0xFFFF;
    inline constexpr std::uint16_t kGravityEnabled = 1;

    // Owns only one generated body's profile selection, never a shared library
    // record. The supplied writer must qualify the body and verify native readback.
    class Lease
    {
    public:
        template<class Writer>
        bool acquire(std::uint16_t current, Writer&& write)
        {
            if (active() || current == 0 || current == kInvalid || !write(kGravityEnabled)) return false;
            _saved = current;
            return true;
        }

        template<class Writer>
        bool restore(Writer&& write)
        {
            if (!active()) return true;
            if (!write(_saved)) return false;
            reset();
            return true;
        }

        bool active() const { return _saved != kInvalid; }
        std::uint16_t saved() const { return _saved; }
        // World loss/body destruction invalidates ownership without touching
        // an old world, recycled body ID or motion slot.
        void reset() { _saved = kInvalid; }

    private:
        std::uint16_t _saved{ kInvalid };
    };
}
