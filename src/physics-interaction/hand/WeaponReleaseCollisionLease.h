#pragma once
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include <cmath>
#include <cstdint>

namespace rock::weapon_release_collision_lease
{
    struct Identity {
        std::uintptr_t world{}, collisionObject{};
        std::uint32_t body{0x7FFF'FFFFu};
        bool operator==(const Identity&) const = default;
        bool valid() const { return world && collisionObject && body != 0x7FFF'FFFFu; }
    };

    // The lease follows one exact generated weapon through held -> dropped
    // layer changes. Only ROCK's two hand compounds are excluded; world,
    // clutter, NPCs and every other weapon retain their normal contacts.
    inline bool suppressesHandPair(const Identity& expected, const Identity& live,
        std::uint32_t weaponLayer, std::uint32_t otherLayer)
    {
        using namespace collision_layer_policy;
        return expected.valid() && expected == live &&
            (weaponLayer == ROCK_LAYER_DYNAMIC_WEAPON_PROXY || weaponLayer == ROCK_LAYER_EQUIPPED_DROP_PROXY) &&
            isDynamicHandInteractionLayer(otherLayer);
    }
    class State
    {
    public:
        bool arm(Identity identity, float seconds)
        {
            if (!identity.valid() ||
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
