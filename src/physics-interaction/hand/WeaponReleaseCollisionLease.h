#pragma once
#include <cmath>
#include <cstdint>

namespace rock::weapon_release_collision_lease
{
    struct Identity {
        std::uintptr_t world{}, collisionObject{};
        std::uint32_t body{0x7FFF'FFFFu};
        bool operator==(const Identity&) const = default;
    };
    class State
    {
    public:
        bool arm(Identity identity, float seconds)
        {
            if (!identity.world || !identity.collisionObject || identity.body == 0x7FFF'FFFFu ||
                !std::isfinite(seconds) || seconds < 0.0f) return false;
            _identity = identity;
            _remaining = seconds;
            return true;
        }
        bool active() const { return _remaining > 0.0f; }
        bool advance(Identity current, float dt)
        {
            if (!(current == _identity)) { reset(); return false; }
            if (std::isfinite(dt) && dt > 0.0f) _remaining = dt < _remaining ? _remaining - dt : 0.0f;
            return active();
        }
        void reset() { _identity = {}; _remaining = 0.0f; }
    private:
        Identity _identity{};
        float _remaining{};
    };
}
