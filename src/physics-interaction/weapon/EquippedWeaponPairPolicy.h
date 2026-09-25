#pragma once

#include <array>
#include <cstdint>

namespace rock::equipped_weapon_pair_policy
{
    struct CollisionOwner { std::uint32_t body{0x7FFFFFFFu}, hands{}; };
    // Each hand suppresses its own equipped weapon proxy, including while
    // that proxy is being rebuilt. No hand can borrow the other gun's body.
    constexpr std::array<CollisionOwner, 2> collisionOwners(CollisionOwner native,
        const std::array<CollisionOwner, 2>& physical) noexcept
    {
        std::array<CollisionOwner, 2> result{};
        for (unsigned hand = 0; hand < result.size(); ++hand) {
            const auto bit = 1u << hand;
            if (native.hands & bit) result[hand] = {native.body, bit};
            for (const auto source : physical) if (source.hands & bit) result[hand] = {source.body, bit};
        }
        return result;
    }
}
